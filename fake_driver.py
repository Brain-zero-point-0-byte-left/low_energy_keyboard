import asyncio
import keyboard
from bleak import BleakScanner, BleakClient
from time import sleep
import keyboard


keyMapping = {
    1: (keyboard.press, 'left'),
    2: (keyboard.press, 'down'),
    3: (keyboard.press, 'right'),
    4: (keyboard.press, 'up'),
}

def receive(buffer: bytes):
    for byte in buffer:
        code = byte
#        print(code, keyMapping.keys())
        if code in keyMapping.keys():
            keyMapping[code][0](keyMapping[code][1])

def debug(b: bytes):
    print(b)



address = "88:25:83:f2:92:2b"
char_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"

def notification_handler(sender, data):
    """Simple notification handler which prints the data received."""
    debug(data)
    receive(data)

async def loop():
    while True:
        await asyncio.sleep(1)

async def main(address):
    async with BleakClient(address) as client:
        print("connected")
        await client.start_notify(char_uuid, notification_handler)
        await loop()
        await client.stop_notify(char_uuid)
        
asyncio.run(main(address))
