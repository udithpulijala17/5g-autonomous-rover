import asyncio
import csv
from datetime import datetime, timezone

from bleak import BleakClient


DEVICE_ADDRESS = "C0:08:BB:CB:CB:45"
NOTIFY_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

OUTPUT_FILE = "logs/imu_ble_raw.csv"


def notification_handler(sender, data: bytearray) -> None:
    timestamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
    hex_data = " ".join(f"{byte:02X}" for byte in data)

    print(f"{timestamp} | {hex_data}")

    with open(OUTPUT_FILE, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, hex_data])


async def main() -> None:
    print(f"Connecting to {DEVICE_ADDRESS}...")

    client = BleakClient(DEVICE_ADDRESS)

    try:
        await client.connect()

        if not client.is_connected:
            raise RuntimeError("BLE connection failed.")

        print("Connected.")
        print(f"Subscribing to {NOTIFY_UUID}...")

        await client.start_notify(NOTIFY_UUID, notification_handler)

        print("Receiving WT901 packets.")
        print("Press Ctrl+C to stop.")

        while True:
            await asyncio.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping capture...")

    finally:
        if client.is_connected:
            try:
                await client.stop_notify(NOTIFY_UUID)
            except Exception:
                pass

            await client.disconnect()

        print("Disconnected.")


if __name__ == "__main__":
    asyncio.run(main())
    
