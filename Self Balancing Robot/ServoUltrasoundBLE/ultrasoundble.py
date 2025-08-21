import asyncio
from bleak import BleakClient

DEVICE_ADDRESS = "XX:XX:XX:XX:XX:XX"  # Replace with your actual Arduino BLE MAC
CHARACTERISTIC_UUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC"

async def notification_handler(sender, data):
    print(f"Received: {data.decode()}")  # Should print "Hello"

async def connect_ble():
    async with BleakClient(DEVICE_ADDRESS) as client:
        if not await client.is_connected():
            print("❌ Failed to connect!")
            return

        print(f"✅ Connected to {DEVICE_ADDRESS}!")

        # Enable notifications
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

        while True:
            await asyncio.sleep(1)  # Keep listening for notifications

asyncio.run(connect_ble())
