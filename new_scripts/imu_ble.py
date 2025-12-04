import asyncio
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from array import array

# Import the reusable IMU parser library
from imu_parser import Cmd_RxUnpack

# Characteristic UUID of the device
# par_notification_characteristic="0000ae02-0000-1000-8000-00805f9b34fb"
par_notification_characteristic=0x0007
# Characteristic UUID of the device (with write attribute Write)
# par_write_characteristic="0000ae01-0000-1000-8000-00805f9b34fb"
par_write_characteristic=0x0005

par_device_addr="AC:25:DD:6E:69:4D" # MAC address of the device You need to fill in the mac address of the device here

# Global IMU data array
imu_dat = array('f',[0.0 for i in range(0,34)])

# ========== IMU Data Callbacks ==========

def on_config_ack():
    """Called when configuration acknowledgment is received (0x12)"""
    print("Configuration set successfully.")

def on_reporting_enabled_ack():
    """Called when proactive reporting enabled acknowledgment is received (0x19)"""
    print("Proactive reporting enabled.")

def on_ble_stay_connected_ack():
    """Called when BLE stay connected acknowledgment is received (0x29)"""
    print("BLE stay connected enabled.")

def on_ble_highspeed_ack():
    """Called when BLE high-speed communication acknowledgment is received (0x46)"""
    print("BLE high-speed communication enabled.")

def on_packet_header(tag, timestamp_ms):
    """Called when packet header is parsed"""
    print("\n subscribe tag: 0x%04x"%tag)
    print(" ms: ", timestamp_ms)

def on_accel_no_gravity(ax, ay, az):
    """Called when acceleration data without gravity is received"""
    global imu_dat
    print("\taX: %.3f"%ax); # Acceleration ax without gravity
    print("\taY: %.3f"%ay); # Acceleration ay without gravity
    print("\taZ: %.3f"%az); # Acceleration az without gravity

    imu_dat[0] = float(ax)
    imu_dat[1] = float(ay)
    imu_dat[2] = float(az)

def on_accel_with_gravity(ax, ay, az, magnitude):
    """Called when acceleration data with gravity is received"""
    global imu_dat
    print("\tAX: %.3f"%ax) # Acceleration AX with gravity
    print("\tAY: %.3f"%ay) # Acceleration AY gravity
    print("\tAZ: %.3f"%az) # Acceleration AZ gravity

    imu_dat[3] = float(ax)
    imu_dat[4] = float(ay)
    imu_dat[5] = float(az)

def on_gyroscope(gx, gy, gz):
    """Called when gyroscope data is received"""
    global imu_dat
    print("\tGX: %.3f"%gx) # Angular velocity GX
    print("\tGY: %.3f"%gy) # Angular velocity GY
    print("\tGZ: %.3f"%gz) # Angular velocity GZ

    imu_dat[6] = float(gx)
    imu_dat[7] = float(gy)
    imu_dat[8] = float(gz)

def on_magnetometer(cx, cy, cz, magnitude):
    """Called when magnetometer data is received"""
    global imu_dat
    print("\tCX: %.3f"%cx); # Magnetic field data CX
    print("\tCY: %.3f"%cy); # Magnetic field data CY
    print("\tCZ: %.3f"%cz); # Magnetic field data CZ

    imu_dat[9] = float(cx)
    imu_dat[10] = float(cy)
    imu_dat[11] = float(cz)

def on_environmental(temperature, air_pressure, height):
    """Called when environmental data is received"""
    global imu_dat
    print("\ttemperature: %.2f"%temperature) # temperature
    print("\tairPressure: %.3f"%air_pressure); # air pressure
    print("\theight: %.3f"%height); # high

    imu_dat[12] = float(temperature)
    imu_dat[13] = float(air_pressure)
    imu_dat[14] = float(height)

def on_quaternion(w, x, y, z):
    """Called when quaternion data is received"""
    global imu_dat
    print("\tw: %.3f"%w); # Quaternion w
    print("\tx: %.3f"%x); # Quaternion x
    print("\ty: %.3f"%y); # Quaternion y
    print("\tz: %.3f"%z); # Quaternion z

    imu_dat[15] = float(w)
    imu_dat[16] = float(x)
    imu_dat[17] = float(y)
    imu_dat[18] = float(z)

def on_euler_angles(roll, pitch, yaw):
    """Called when Euler angle data is received"""
    global imu_dat
    print("\tangleX: %.3f"%roll); # Euler angle x (roll)
    print("\tangleY: %.3f"%pitch); # Euler angle y (pitch)
    print("\tangleZ: %.3f"%yaw); # Euler angle z (yaw)

    imu_dat[19] = float(roll)
    imu_dat[20] = float(pitch)
    imu_dat[21] = float(yaw)

def on_unknown_command(command_id):
    """Called when unknown command ID is received"""
    print(f"[error] data head not define: 0x{command_id:02X}")

# Create callbacks dictionary for the IMU parser
imu_callbacks = {
    'config_ack': on_config_ack,
    'reporting_enabled_ack': on_reporting_enabled_ack,
    'ble_stay_connected_ack': on_ble_stay_connected_ack,
    'ble_highspeed_ack': on_ble_highspeed_ack,
    'packet_header': on_packet_header,
    'accel_no_gravity': on_accel_no_gravity,
    'accel_with_gravity': on_accel_with_gravity,
    'gyroscope': on_gyroscope,
    'magnetometer': on_magnetometer,
    'environmental': on_environmental,
    'quaternion': on_quaternion,
    'euler_angles': on_euler_angles,
    'unknown_command': on_unknown_command,
}

# Listening callback function, here is the print message
def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    """BLE notification handler - receives data directly from BLE"""
    # BLE delivers the payload directly (no serial packet framing needed)
    # Call Cmd_RxUnpack directly with the data
    Cmd_RxUnpack(data, len(data), callbacks=imu_callbacks)

async def main():
    print("Starting scan...")

    device = None
    devices = await BleakScanner.discover(timeout=10.0)
    for d in devices:
        if d.address == par_device_addr:
            device = d
            break
    
    if device is None:
        print(f"could not find device with address '{par_device_addr}'")
        return

    print(f"Found: {device.name}")

    disconnected_event = asyncio.Event()

    def disconnected_callback(client):
        print("Disconnected callback called!")
        disconnected_event.set()

    for attempt in range(5):
        try:
            print(f"connecting to device... (attempt {attempt + 1})")
            async with BleakClient(device, disconnected_callback=disconnected_callback) as client:
                print("Connected")

                # Start notifications
                await client.start_notify(par_notification_characteristic, notification_handler)
                # stay connected 0x29
                await client.write_gatt_char(par_write_characteristic, bytes([0x29]))
                # Try to use Bluetooth high-speed communication features 0x46
                await client.write_gatt_char(par_write_characteristic, bytes([0x46]))

                # Parameter settings
                isCompassOn = 0 #Whether to use magnetic field fusion 0: Not used 1: Used
                barometerFilter = 2
                Cmd_ReportTag = 0x7F # Feature subscription tag
                params = bytearray([0x00 for i in range(0,11)])
                params[0] = 0x12
                params[1] = 5       #Stationary state acceleration threshold
                params[2] = 255     #Static zero return speed (unit cm/s) 0: No return to zero 255: Return to zero immediately
                params[3] = 0       #Dynamic zero return speed (unit cm/s) 0: No return to zero
                params[4] = ((barometerFilter&3)<<1) | (isCompassOn&1)
                params[5] = 60      #The transmission frame rate of data actively reported [value 0-250HZ], 0 means 0.5HZ
                params[6] = 1       #Gyroscope filter coefficient [value 0-2], the larger the value, the more stable it is but the worse the real-time performance.
                params[7] = 3       #Accelerometer filter coefficient [value 0-4], the larger the value, the more stable it is but the worse the real-time performance.
                params[8] = 5       #Magnetometer filter coefficient [value 0-9], the larger the value, the more stable it is but the worse the real-time performance.
                params[9] = Cmd_ReportTag&0xff
                params[10] = (Cmd_ReportTag>>8)&0xff
                await client.write_gatt_char(par_write_characteristic, params)

                # Enable proactive reporting
                await client.write_gatt_char(par_write_characteristic, bytes([0x19]))

                # Add a loop so that the program does not exit while receiving data
                while not disconnected_event.is_set():
                    await asyncio.sleep(1.0)

                # await disconnected_event.wait() # Sleep until device disconnects, with delay. Here is the listening device until disconnected
                # await client.stop_notify(par_notification_characteristic)
        except Exception as e:
                print(f"Connection failed: {e}")
                await asyncio.sleep(3)
    else:
        print("Failed to connect after 5 attempts")

asyncio.run(main())