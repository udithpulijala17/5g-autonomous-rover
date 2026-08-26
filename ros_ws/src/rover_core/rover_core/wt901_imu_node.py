from __future__ import annotations

import asyncio
import math
import queue
import threading
import time
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
# IMU CALIBRATION / FILTERING
# ============================================================

# Number of stationary gyro samples used for startup bias calibration.
CALIBRATION_SAMPLES = 300

# Maximum time allowed for startup calibration.
CALIBRATION_TIMEOUT = 8.0

# Gyro low-pass filter coefficient.
# Smaller = smoother, larger = faster response.
GYRO_ALPHA = 0.20

# Samples with gyro magnitude above this threshold are ignored
# during startup calibration.
# Unit: rad/s
STATIONARY_GYRO_THRESHOLD = 0.08

# Reject integration steps larger than this.
# This prevents a BLE scheduling gap from creating a false
# large yaw jump.
MAX_DT = 0.20


# ============================================================
# PARSED WT901 DATA
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
# LOW-LEVEL WT901 DECODING
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

    Expected 20-byte frame:

        55 61
        AX AY AZ
        GX GY GZ
        ROLL PITCH YAW

    All nine values are signed int16.
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

    (
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        roll,
        pitch,
        yaw,
    ) = values

    return WT901Data(
        # Accelerometer: ±16 g
        ax_g=ax / 32768.0 * 16.0,
        ay_g=ay / 32768.0 * 16.0,
        az_g=az / 32768.0 * 16.0,

        # Gyroscope: ±2000 deg/s
        gx_dps=gx / 32768.0 * 2000.0,
        gy_dps=gy / 32768.0 * 2000.0,
        gz_dps=gz / 32768.0 * 2000.0,

        # Euler angles
        roll_deg=roll / 32768.0 * 180.0,
        pitch_deg=pitch / 32768.0 * 180.0,
        yaw_deg=yaw / 32768.0 * 180.0,
    )


# ============================================================
# EULER -> QUATERNION
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


def normalize_angle_deg(angle: float) -> float:
    """Normalize angle to [-180, 180)."""

    return (angle + 180.0) % 360.0 - 180.0


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
        # BLE -> ROS queue
        # ----------------------------------------------------

        self.data_queue: queue.Queue[WT901Data] = queue.Queue()

        # ----------------------------------------------------
        # Shutdown
        # ----------------------------------------------------

        self.shutdown_event = threading.Event()

        # ----------------------------------------------------
        # Startup gyro calibration
        # ----------------------------------------------------

        self.calibrating = True
        self.calibration_started = time.monotonic()

        self.calibration_count = 0

        self.gx_bias_dps = 0.0
        self.gy_bias_dps = 0.0
        self.gz_bias_dps = 0.0

        self.gx_bias_sum = 0.0
        self.gy_bias_sum = 0.0
        self.gz_bias_sum = 0.0

        # ----------------------------------------------------
        # Filter state
        # ----------------------------------------------------

        self.gx_filtered = 0.0
        self.gy_filtered = 0.0
        self.gz_filtered = 0.0

        self.filter_initialized = False

        # ----------------------------------------------------
        # Integrated yaw
        #
        # IMPORTANT:
        # We intentionally DO NOT use the WT901's reported
        # magnetic/AHRS yaw for localization.
        #
        # Instead:
        #
        # calibrated gyro-Z -> filter -> integrate -> yaw
        # ----------------------------------------------------

        self.yaw_integrated_deg = 0.0
        self.last_gyro_time: float | None = None

        # ----------------------------------------------------
        # ROS publishing timer
        # ----------------------------------------------------

        self.publish_timer = self.create_timer(
            0.005,
            self.publish_pending_data,
        )

        # ----------------------------------------------------
        # BLE worker thread
        # ----------------------------------------------------

        self.ble_thread = threading.Thread(
            target=self.ble_thread_entry,
            name="wt901_ble_thread",
            daemon=True,
        )

        self.ble_thread.start()

        self.get_logger().info(
            "WT901 stabilized BLE ROS 2 node initialized."
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
                # Discover
                # ------------------------------------------------

                self.get_logger().info(
                    f"Scanning for {WT901_NAME}..."
                )

                device = (
                    await BleakScanner.find_device_by_name(
                        WT901_NAME,
                        timeout=BLE_SCAN_TIMEOUT,
                    )
                )

                if device is None:

                    self.get_logger().warning(
                        f"{WT901_NAME} not found. "
                        f"Retrying in "
                        f"{BLE_RECONNECT_DELAY}s."
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
                # Enable notifications
                # ------------------------------------------------

                await client.start_notify(
                    NOTIFY_UUID,
                    notification_callback,
                )

                self.get_logger().info(
                    "FFE4 notifications enabled."
                )

                # ------------------------------------------------
                # Keep connection alive
                # ------------------------------------------------

                while (
                    client.is_connected
                    and not self.shutdown_event.is_set()
                ):

                    await asyncio.sleep(0.5)

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
    # GYRO CALIBRATION
    # ========================================================

    def process_calibration(
        self,
        data: WT901Data,
    ) -> None:

        if not self.calibrating:
            return

        elapsed = (
            time.monotonic()
            - self.calibration_started
        )

        # ----------------------------------------------------
        # Timeout
        # ----------------------------------------------------

        if elapsed > CALIBRATION_TIMEOUT:

            if self.calibration_count == 0:

                self.get_logger().error(
                    "IMU calibration failed: "
                    "no valid stationary samples."
                )

                return

            self.finish_calibration()

            return

        # ----------------------------------------------------
        # Convert deg/s -> rad/s for stationary test
        # ----------------------------------------------------

        d2r = math.pi / 180.0

        gx = data.gx_dps * d2r
        gy = data.gy_dps * d2r
        gz = data.gz_dps * d2r

        # ----------------------------------------------------
        # Ignore samples where the rover is moving
        # ----------------------------------------------------

        if (
            abs(gx) > STATIONARY_GYRO_THRESHOLD
            or abs(gy) > STATIONARY_GYRO_THRESHOLD
            or abs(gz) > STATIONARY_GYRO_THRESHOLD
        ):

            return

        # ----------------------------------------------------
        # Accumulate bias
        # ----------------------------------------------------

        self.gx_bias_sum += data.gx_dps
        self.gy_bias_sum += data.gy_dps
        self.gz_bias_sum += data.gz_dps

        self.calibration_count += 1

        # ----------------------------------------------------
        # Complete calibration
        # ----------------------------------------------------

        if self.calibration_count >= CALIBRATION_SAMPLES:

            self.finish_calibration()

    def finish_calibration(self) -> None:

        if self.calibration_count <= 0:

            self.get_logger().error(
                "Unable to calculate gyro bias."
            )

            return

        n = float(self.calibration_count)

        self.gx_bias_dps = (
            self.gx_bias_sum / n
        )

        self.gy_bias_dps = (
            self.gy_bias_sum / n
        )

        self.gz_bias_dps = (
            self.gz_bias_sum / n
        )

        self.calibrating = False

        # Reset yaw integration after calibration.
        self.yaw_integrated_deg = 0.0
        self.last_gyro_time = None

        # Reset filter so the first post-calibration
        # sample initializes it cleanly.
        self.filter_initialized = False

        self.get_logger().info(
            "================================================"
        )

        self.get_logger().info(
            "WT901 gyro calibration complete."
        )

        self.get_logger().info(
            f"Gyro bias X: "
            f"{self.gx_bias_dps:.6f} deg/s"
        )

        self.get_logger().info(
            f"Gyro bias Y: "
            f"{self.gy_bias_dps:.6f} deg/s"
        )

        self.get_logger().info(
            f"Gyro bias Z: "
            f"{self.gz_bias_dps:.6f} deg/s"
        )

        self.get_logger().info(
            "WT901 magnetic yaw will NOT be used."
        )

        self.get_logger().info(
            "Yaw will be obtained by integrating "
            "calibrated gyro-Z."
        )

        self.get_logger().info(
            "================================================"
        )

    # ========================================================
    # ROS PUBLISH
    # ========================================================

    def publish_pending_data(self) -> None:

        while True:

            try:

                data = (
                    self.data_queue.get_nowait()
                )

            except queue.Empty:

                break

            # ------------------------------------------------
            # Calibration phase
            # ------------------------------------------------

            if self.calibrating:

                if not hasattr(
                    self,
                    "_calibration_logged"
                ):

                    self.get_logger().info(
                        "Calibrating IMU. "
                        "KEEP THE ROVER COMPLETELY STILL."
                    )

                    self._calibration_logged = True

                self.process_calibration(
                    data
                )

                continue

            # ------------------------------------------------
            # Correct gyro bias
            # ------------------------------------------------

            gx_dps = (
                data.gx_dps
                - self.gx_bias_dps
            )

            gy_dps = (
                data.gy_dps
                - self.gy_bias_dps
            )

            gz_dps = (
                data.gz_dps
                - self.gz_bias_dps
            )

            # ------------------------------------------------
            # Low-pass gyro
            # ------------------------------------------------

            if not self.filter_initialized:

                self.gx_filtered = gx_dps
                self.gy_filtered = gy_dps
                self.gz_filtered = gz_dps

                self.filter_initialized = True

            else:

                self.gx_filtered = (
                    GYRO_ALPHA * gx_dps
                    + (1.0 - GYRO_ALPHA)
                    * self.gx_filtered
                )

                self.gy_filtered = (
                    GYRO_ALPHA * gy_dps
                    + (1.0 - GYRO_ALPHA)
                    * self.gy_filtered
                )

                self.gz_filtered = (
                    GYRO_ALPHA * gz_dps
                    + (1.0 - GYRO_ALPHA)
                    * self.gz_filtered
                )

            # ------------------------------------------------
            # Integrate gyro-Z
            #
            # IMPORTANT:
            # This replaces the WT901's reported yaw.
            # ------------------------------------------------

            now = time.monotonic()

            if self.last_gyro_time is None:

                self.last_gyro_time = now

            else:

                dt = (
                    now
                    - self.last_gyro_time
                )

                if (
                    dt > 0.0
                    and dt < MAX_DT
                ):

                    self.yaw_integrated_deg += (
                        self.gz_filtered * dt
                    )

                self.last_gyro_time = now

            self.yaw_integrated_deg = (
                normalize_angle_deg(
                    self.yaw_integrated_deg
                )
            )

            # ------------------------------------------------
            # IMPORTANT:
            # Do NOT use data.yaw_deg here.
            # ------------------------------------------------

            relative_yaw_deg = (
                self.yaw_integrated_deg
            )

            # ------------------------------------------------
            # Roll + pitch:
            # continue using WT901 AHRS values.
            #
            # Yaw:
            # use integrated gyro-Z.
            # ------------------------------------------------

            (
                qx,
                qy,
                qz,
                qw,
            ) = euler_to_quaternion(
                data.roll_deg,
                data.pitch_deg,
                relative_yaw_deg,
            )

            # ------------------------------------------------
            # ROS Imu message
            # ------------------------------------------------

            msg = Imu()

            msg.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            msg.header.frame_id = (
                IMU_FRAME
            )

            # ------------------------------------------------
            # Acceleration
            #
            # Published for completeness.
            # Current EKF configuration does not use it.
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
            # Filtered gyro
            # ------------------------------------------------

            d2r = math.pi / 180.0

            msg.angular_velocity.x = (
                self.gx_filtered
                * d2r
            )

            msg.angular_velocity.y = (
                self.gy_filtered
                * d2r
            )

            msg.angular_velocity.z = (
                self.gz_filtered
                * d2r
            )

            # ------------------------------------------------
            # Orientation
            # ------------------------------------------------

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            # ------------------------------------------------
            # Covariance
            # ------------------------------------------------

            msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.02,
            ]

            msg.angular_velocity_covariance = [
                0.005, 0.0, 0.0,
                0.0, 0.005, 0.0,
                0.0, 0.0, 0.01,
            ]

            msg.linear_acceleration_covariance = [
                0.20, 0.0, 0.0,
                0.0, 0.20, 0.0,
                0.0, 0.0, 0.20,
            ]

            self.publisher.publish(
                msg
            )

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

    rclpy.init(
        args=args
    )

    node = WT901ImuNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.shutdown()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
