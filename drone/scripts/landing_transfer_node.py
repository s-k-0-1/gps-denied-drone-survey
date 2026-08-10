#!/usr/bin/env python3
# Jetson landing handler — UNCHANGED rsync transfer, PLUS a one-line notify to
# the base station so it can start the docking/charging sequence.
#
# On touchdown this now does TWO things (concurrently):
#   1. notify the base station  -> POST /api/landed  (base station waits 5s,
#      then commands the ESP32 to dock)
#   2. rsync the latest survey folder to the PC  (exactly as before)
#
# Run this the same way you run your existing node.

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ExtendedState
import subprocess
import os
import glob
import threading
import urllib.request

# ── EDIT THESE FOR YOUR SETUP ────────────────────────────────────────────────
# They can also be supplied as environment variables, which is preferable —
# then nothing sensitive lives in the file:
#     PC_USER=me PC_IP=192.168.1.100 DOCK_TOKEN=secret ros2 run ...
PC_USER = os.environ.get("PC_USER", "YOUR_PC_USERNAME")
PC_IP = os.environ.get("PC_IP", "192.168.1.100")   # ground PC IP — find it with: hostname -I
PC_PORT = os.environ.get("PC_PORT", "22")          # SSH port on the ground PC
PC_DEST_PATH = os.environ.get(
    "PC_DEST_PATH", "/home/YOUR_PC_USERNAME/gps-denied-drone-survey/drone_photos/")
SURVEY_ROOT = os.environ.get("SURVEY_ROOT", "/media/jetson/ROS2_SSD/survey/")

# ── base station notify ──
BASE_URL = os.environ.get("BASE_URL", f"http://{PC_IP}:8000")
DOCK_TOKEN = os.environ.get("DOCK_TOKEN", "CHANGE_ME")  # must match IROC_TOKEN on the base
                                                        # station AND DOCK_TOKEN on the ESP32


class LandingTransfer(Node):
    def __init__(self):
        super().__init__('landing_transfer_node')
        self.last_state = None
        self.transferred_this_landing = False
        self.sub = self.create_subscription(
            ExtendedState, '/mavros/extended_state', self.cb, 10)
        self.get_logger().info("Waiting for drone to land...")

    def cb(self, msg):
        state = msg.landed_state
        if state in (4, 1):
            if not self.transferred_this_landing:
                self.get_logger().info("Drone landed!")
                # 1) tell the base station immediately (fires the docking timer)
                self.notify_base_station()
                # 2) transfer the survey data (unchanged)
                self.run_transfer()
                self.transferred_this_landing = True
        elif state == 2:
            self.transferred_this_landing = False
        self.last_state = state

    def notify_base_station(self):
        """Fire-and-forget POST so it never blocks/holds up the rsync."""
        def _post():
            url = f"{BASE_URL}/api/landed?token={DOCK_TOKEN}"
            try:
                req = urllib.request.Request(url, data=b"", method="POST")
                with urllib.request.urlopen(req, timeout=4) as r:
                    r.read()
                self.get_logger().info("Base station notified (docking armed).")
            except Exception as e:
                self.get_logger().error(f"Base station notify failed: {e}")
        threading.Thread(target=_post, daemon=True).start()

    def notify_transfer_done(self):
        """Tell the base station the photos are in → it auto-runs the pipeline."""
        def _post():
            url = f"{BASE_URL}/api/transfer_done?token={DOCK_TOKEN}"
            try:
                req = urllib.request.Request(url, data=b"", method="POST")
                with urllib.request.urlopen(req, timeout=6) as r:
                    r.read()
                self.get_logger().info("Base station notified — pipeline auto-started.")
            except Exception as e:
                self.get_logger().error(f"transfer_done notify failed: {e}")
        threading.Thread(target=_post, daemon=True).start()

    def get_latest_folder(self):
        folders = glob.glob(os.path.join(SURVEY_ROOT, "*survey_*"))
        folders = [f for f in folders if os.path.isdir(f)]
        if not folders:
            return None
        return max(folders, key=os.path.getmtime)

    def run_transfer(self):
        latest_folder = self.get_latest_folder()
        if latest_folder is None:
            self.get_logger().error("No survey folder found to transfer.")
            return
        folder_name = os.path.basename(latest_folder.rstrip('/'))
        self.get_logger().info(f"Sharing data: {folder_name} -> PC...")
        # NOTE: NO --delete. rsync only ADDS/UPDATES files in drone_photos/ —
        # it never deletes anything, on the PC or anywhere else.
        cmd = [
            "rsync", "-avz", "--partial",
            "-e", f"ssh -p {PC_PORT}",
            latest_folder + "/",
            f"{PC_USER}@{PC_IP}:{PC_DEST_PATH}"
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
            if result.returncode == 0:
                self.get_logger().info(f"Data transfer complete: {folder_name}")
                self.notify_transfer_done()          # → base station auto-runs the pipeline
            else:
                self.get_logger().error(f"rsync failed: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"Transfer error: {e}")


def main():
    rclpy.init()
    node = LandingTransfer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
