from __future__ import annotations

import asyncio
import math
import queue
import threading
from dataclasses import dataclass

from bleak import BleakClient, BleakScanner

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


# ============================================================
# WT901 CONFIGURATION
# ============================================================

WT901_NAME = "WT901BLE68"

NOTIFY_UUID = (
    "0000ffe4-0000-1000-8000-00805f9a34fb"
)

IMU_FRAME = "imu_link"
IMU_TOPIC = "/imu/data"

GRAVITY = 9.80665

BLE_SCAN_TIMEOUT = 10.0
BLE_RECONNECT_DELAY = 3.0


# ============================================================
# PARSED IMU DATA
# ============================================================

@dataclass
class WT901Data:
    ax_g: float
    ay_g: float
    az_g: float

    gx_dps: float
    gy_dps: float
    gz_dps: float

    roll_deg: float
    pitch_deg: float
    yaw_deg: float


# ============================================================
# WT901 PARSER
# ============================================================

def int16_le(lo: int, hi: int) -> int:
    """Decode signed little-endian int16."""
    value = lo | (hi << 8)

    if value & 0x8000:
        value -= 0x10000

    return value


def parse_0x61(packet: bytes) -> WT901Data:
    """
    Parse a WT901 0x61 frame.

    Expected:
        20 bytes
        55 61 + 9 signed int16 values

    Order:
        AX AY AZ
        GX GY GZ
        Roll Pitch Yaw
    """

    if len(packet) != 20:
        raise ValueError(
            f"Invalid packet length: {len(packet)} bytes"
        )

    if packet[0] != 0x55 or packet[1] != 0x61:
        raise ValueError(
            "Invalid WT901 0x61 header: "
            f"{packet[0]:02X} {packet[1]:02X}"
        )

    values = [
        int16_le(packet[i], packet[i + 1])
        for i in range(2, 20, 2)
    ]

    ax, ay, az, gx, gy, gz, roll, pitch, yaw = values

    return WT901Data(
        ax_g=ax / 32768.0 * 16.0,
        ay_g=ay / 32768.0 * 16.0,
        az_g=az / 32768.0 * 16.0,

        gx_dps=gx / 32768.0 * 2000.0,
        gy_dps=gy / 32768.0 * 2000.0,
        gz_dps=gz / 32768.0 * 2000.0,

        roll_deg=roll / 32768.0 * 180.0,
        pitch_deg=pitch / 32768.0 * 180.0,
        yaw_deg=yaw / 32768.0 * 180.0,
    )


# ============================================================
# EULER → QUATERNION
# ============================================================

def euler_to_quaternion(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> tuple[float, float, float, float]:

    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = (
        cr * cp * cy
        + sr * sp * sy
    )

    x = (
        sr * cp * cy
        - cr * sp * sy
    )

    y = (
        cr * sp * cy
        + sr * cp * sy
    )

    z = (
        cr * cp * sy
        - sr * sp * cy
    )

    return x, y, z, w


# ============================================================
# ROS 2 NODE
# ============================================================

class WT901ImuNode(Node):

    def __init__(self) -> None:

        super().__init__("wt901_imu_node")

        # ----------------------------------------------------
        # ROS publisher
        # ----------------------------------------------------

        self.publisher = self.create_publisher(
            Imu,
            IMU_TOPIC,
            20,
        )

        # ----------------------------------------------------
        # Queue between BLE thread and ROS thread
        # ----------------------------------------------------

        self.data_queue: queue.Queue[WT901Data] = queue.Queue()

        # ----------------------------------------------------
        # Shutdown control
        # ----------------------------------------------------

        self.shutdown_event = threading.Event()

        # ----------------------------------------------------
        # ROS timer drains BLE data queue
        # ----------------------------------------------------

        self.publish_timer = self.create_timer(
            0.005,
            self.publish_pending_data,
        )

        # ----------------------------------------------------
        # BLE thread
        # ----------------------------------------------------

        self.ble_thread = threading.Thread(
            target=self.ble_thread_entry,
            name="wt901_ble_thread",
            daemon=True,
        )

        self.ble_thread.start()

        self.get_logger().info(
            "WT901 BLE ROS 2 node initialized."
        )


    # ========================================================
    # BLE THREAD
    # ========================================================

    def ble_thread_entry(self) -> None:

        try:

            asyncio.run(
                self.ble_worker()
            )

        except Exception as exc:

            if not self.shutdown_event.is_set():

                self.get_logger().error(
                    f"BLE thread stopped: {exc}"
                )


    async def ble_worker(self) -> None:

        while not self.shutdown_event.is_set():

            client = None

            try:

                # ------------------------------------------------
                # Discover WT901 by name
                # ------------------------------------------------

                self.get_logger().info(
                    f"Scanning for {WT901_NAME}..."
                )

                device = await BleakScanner.find_device_by_name(
                    WT901_NAME,
                    timeout=BLE_SCAN_TIMEOUT,
                )

                if device is None:

                    self.get_logger().warning(
                        f"{WT901_NAME} not found. "
                        f"Retrying in {BLE_RECONNECT_DELAY}s."
                    )

                    await asyncio.sleep(
                        BLE_RECONNECT_DELAY
                    )

                    continue

                self.get_logger().info(
                    f"Found {WT901_NAME} "
                    f"at {device.address}"
                )

                # ------------------------------------------------
                # Connect
                # ------------------------------------------------

                client = BleakClient(device)

                await client.connect()

                if not client.is_connected:

                    raise RuntimeError(
                        "BLE connection failed."
                    )

                self.get_logger().info(
                    "Connected to WT901."
                )

                # ------------------------------------------------
                # Notification callback
                # ------------------------------------------------

                def notification_callback(
                    sender: int,
                    data: bytearray,
                ) -> None:

                    try:

                        parsed = parse_0x61(
                            bytes(data)
                        )

                        self.data_queue.put(
                            parsed
                        )

                    except ValueError as exc:

                        self.get_logger().warning(
                            f"Invalid WT901 packet: {exc}"
                        )

                # ------------------------------------------------
                # Start FFE4 notifications
                # ------------------------------------------------

                await client.start_notify(
                    NOTIFY_UUID,
                    notification_callback,
                )

                self.get_logger().info(
                    "FFE4 notifications enabled."
                )

                # ------------------------------------------------
                # Keep BLE connection alive
                # ------------------------------------------------

                while (
                    client.is_connected
                    and not self.shutdown_event.is_set()
                ):

                    await asyncio.sleep(0.5)

                # ------------------------------------------------
                # Connection lost
                # ------------------------------------------------

                if not self.shutdown_event.is_set():

                    self.get_logger().warning(
                        "WT901 BLE connection lost."
                    )

            except Exception as exc:

                if not self.shutdown_event.is_set():

                    self.get_logger().error(
                        f"BLE error: {exc}"
                    )

            finally:

                if client is not None:

                    try:

                        if client.is_connected:

                            try:

                                await client.stop_notify(
                                    NOTIFY_UUID
                                )

                            except Exception:
                                pass

                            await client.disconnect()

                    except Exception:
                        pass

                if not self.shutdown_event.is_set():

                    await asyncio.sleep(
                        BLE_RECONNECT_DELAY
                    )


    # ========================================================
    # ROS PUBLISH
    # ========================================================

    def publish_pending_data(self) -> None:

        while True:

            try:

                data = self.data_queue.get_nowait()

            except queue.Empty:

                break

            msg = Imu()

            # ------------------------------------------------
            # Header
            # ------------------------------------------------

            msg.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            msg.header.frame_id = IMU_FRAME

            # ------------------------------------------------
            # Linear acceleration
            # WT901 output is in g
            # ROS expects m/s²
            # ------------------------------------------------

            msg.linear_acceleration.x = (
                data.ax_g * GRAVITY
            )

            msg.linear_acceleration.y = (
                data.ay_g * GRAVITY
            )

            msg.linear_acceleration.z = (
                data.az_g * GRAVITY
            )

            # ------------------------------------------------
            # Angular velocity
            # WT901 output is °/s
            # ROS expects rad/s
            # ------------------------------------------------

            deg_to_rad = math.pi / 180.0

            msg.angular_velocity.x = (
                data.gx_dps * deg_to_rad
            )

            msg.angular_velocity.y = (
                data.gy_dps * deg_to_rad
            )

            msg.angular_velocity.z = (
                data.gz_dps * deg_to_rad
            )

            # ------------------------------------------------
            # Orientation
            # ------------------------------------------------

            (
                qx,
                qy,
                qz,
                qw,
            ) = euler_to_quaternion(
                data.roll_deg,
                data.pitch_deg,
                data.yaw_deg,
            )

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            # ------------------------------------------------
            # Covariance
            #
            # We have not experimentally calibrated these
            # yet, so leave the covariance unspecified.
            # ------------------------------------------------

            msg.orientation_covariance[0] = -1.0

            msg.angular_velocity_covariance[0] = -1.0

            msg.linear_acceleration_covariance[0] = -1.0

            self.publisher.publish(msg)


    # ========================================================
    # CLEAN SHUTDOWN
    # ========================================================

    def shutdown(self) -> None:

        self.get_logger().info(
            "Stopping WT901 BLE node..."
        )

        self.shutdown_event.set()

        if self.ble_thread.is_alive():

            self.ble_thread.join(
                timeout=5.0
            )

        self.get_logger().info(
            "WT901 BLE node stopped."
        )


# ============================================================
# MAIN
# ============================================================

def main(args=None) -> None:

    rclpy.init(args=args)

    node = WT901ImuNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.shutdown()

        node.destroy_node()

        # rclpy may already be shutting down after SIGINT.
        # Only call shutdown if the context is still active.

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
