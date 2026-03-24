# robot_bridge.py
# ================================================================
# Unitree Go2 — ROS2 to FastAPI Bridge
# Reads lowstate (IMU, BMS, Motors) and exposes via REST + WebSocket
# ================================================================

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState

import json
import asyncio
import threading
from fastapi import FastAPI, WebSocket
import uvicorn

# ================================================================
# SHARED STATE
# Acts as a memory bridge between ROS2 thread and FastAPI thread
# ROS2 writes to it → FastAPI reads from it
# ================================================================
latest_data = {}

# ================================================================
# ROS2 NODE
# Subscribes to /lf/lowstate topic and parses incoming messages
# ================================================================
class LowStateSubscriber(Node):
    def __init__(self):
        super().__init__('lowstate_bridge')

        self.sub = self.create_subscription(
            LowState,           # Message type from unitree_go package
            'lf/lowstate',      # Topic name (lf = low frequency, easier to start)
            self.callback,      # Function called every time new data arrives
            10                  # Queue size — how many messages to buffer
        )
        self.get_logger().info('LowState subscriber started, waiting for data...')

    def callback(self, msg):
        """
        Called automatically every time the robot publishes a new lowstate message.
        Parses the ROS2 message into a plain Python dict stored in latest_data.
        """
        global latest_data

        latest_data = {

            # ----------------------------------------------------
            # IMU — Inertial Measurement Unit
            # Orientation, rotation rates, and acceleration of robot body
            # ----------------------------------------------------
            "imu": {
                "quaternion":    list(msg.imu_state.quaternion),     # orientation (x,y,z,w)
                "gyroscope":     list(msg.imu_state.gyroscope),      # rotation rate (rad/s)
                "accelerometer": list(msg.imu_state.accelerometer),  # linear acceleration (m/s²)
                "rpy":           list(msg.imu_state.rpy),            # roll, pitch, yaw (rad)
            },

            # ----------------------------------------------------
            # BMS — Battery Management System
            # Battery health, charge level, and thermal info
            # ----------------------------------------------------
            "bms": {
                "soc":          msg.bms_state.soc,                        # State of charge (battery %)
                "current":      msg.bms_state.current,                    # Current draw (A)
                "cycle":        msg.bms_state.cycle,                      # Charge cycle count
                "temperatures": list(msg.bms_state.temperatures),         # Battery cell temperatures
                "version_high": msg.bms_state.version_high,               # BMS firmware version
                "version_low":  msg.bms_state.version_low,
                "status":       msg.bms_state.status,                     # BMS status flags
            },

            # ----------------------------------------------------
            # MOTORS — All 20 joint motors
            # Position, velocity, torque, and temperature per joint
            # ----------------------------------------------------
            "motors": [
                {
                    "id":          i,
                    "q":           msg.motor_state[i].q,           # Joint angle (rad)
                    "dq":          msg.motor_state[i].dq,          # Joint velocity (rad/s)
                    "tau_est":     msg.motor_state[i].tau_est,     # Estimated torque (Nm)
                    "temperature": msg.motor_state[i].temperature, # Motor temperature (°C)
                }
                for i in range(20)  # Go2 has 12 leg motors + 8 reserve slots
            ],

            # ----------------------------------------------------
            # POWER — Overall power consumption
            # ----------------------------------------------------
            "power_v": msg.power_v,   # Supply voltage (V)
            "power_a": msg.power_a,   # Total current draw (A)
        }


# ================================================================
# FASTAPI APP
# Exposes latest_data over two interfaces:
#   GET  /robot/state     → REST (polling)
#   WS   /ws/robot        → WebSocket (real-time stream)
# ================================================================
app = FastAPI()


@app.get("/robot/state")
def get_state():
    """
    REST endpoint — returns the latest robot state as JSON.
    The other person can poll this URL at any interval they want.
    Example: GET https://your-url/robot/state
    """
    if not latest_data:
        return {"error": "No data received yet. Is the robot connected?"}
    return latest_data


@app.websocket("/ws/robot")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint — pushes latest robot state every 0.1 seconds (10Hz).
    The other person connects once and receives a continuous stream.
    Example: wss://your-url/ws/robot
    """
    await websocket.accept()
    print(f"WebSocket client connected: {websocket.client}")

    try:
        while True:
            await asyncio.sleep(0.1)  # Push rate: 10Hz (change to 0.5 for 2Hz etc.)
            if latest_data:
                await websocket.send_text(json.dumps(latest_data))
    except Exception as e:
        # Client disconnected — clean exit
        print(f"WebSocket client disconnected: {e}")


# ================================================================
# ROS2 SPIN — runs in a background thread
# rclpy.spin() is blocking, so it must run separately
# to avoid freezing the FastAPI server
# ================================================================
def ros_spin():
    """
    Initializes ROS2 and keeps the subscriber node running forever.
    Runs in a daemon thread — will auto-stop when main program exits.
    """
    rclpy.init()
    node = LowStateSubscriber()
    rclpy.spin(node)        # Blocks here, calling callback on every new message
    rclpy.shutdown()


# ================================================================
# ENTRY POINT
# 1. Start ROS2 subscriber in background thread
# 2. Start FastAPI server in main thread
# ================================================================
if __name__ == "__main__":

    # Start ROS2 in background (daemon=True means it stops with the main program)
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    print("ROS2 subscriber thread started...")

    # Start FastAPI in main thread
    # host="0.0.0.0" makes it accessible from other machines on the network
    print("Starting FastAPI server on http://0.0.0.0:8000")
    print("REST endpoint:      http://localhost:8000/robot/state")
    print("WebSocket endpoint: ws://localhost:8000/ws/robot")
    uvicorn.run(app, host="0.0.0.0", port=8000)

