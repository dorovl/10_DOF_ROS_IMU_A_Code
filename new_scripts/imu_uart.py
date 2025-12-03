import serial
import time

# Import the reusable IMU parser library
from imu_parser import Cmd_GetPkt, Cmd_PackAndTx

# Set the correct serial port parameters------------------------
ser_port = "COM18"     #This needs to be replaced with the corresponding serial port number. For Windows systems, it is written as COMx. If it is Linux, it needs to be adjusted according to the system used, such as /dev/ttyUSBx or /dev/ttySx.
ser_baudrate = 115200 # 串Port baud rate
ser_timeout = 2 # Serial port operation timeout time

# Open the serial port
ser = serial.Serial(ser_port, ser_baudrate, timeout=ser_timeout)

# ========== IMU Data Callbacks ==========
# These callbacks are invoked by the imu_parser when data is received

def on_packet_received(hex_string, length):
    """Called when a complete packet is received"""
    print(f"U-Rx[Len={length}]:{hex_string}")

def on_packet_header(tag, timestamp_ms):
    """Called when packet header is parsed"""
    print("\n subscribe tag: 0x%04x"%tag)
    print(" ms: ", timestamp_ms)

def on_accel_no_gravity(ax, ay, az):
    """Called when acceleration data without gravity is received"""
    print("\taX: %.3f"%ax);  # Acceleration ax without gravity
    print("\taY: %.3f"%ay);  # Acceleration ay without gravity
    print("\taZ: %.3f"%az);  # Acceleration az without gravity

def on_accel_with_gravity(ax, ay, az, magnitude):
    """Called when acceleration data with gravity is received"""
    print("\tAX: %.3f"%ax)  # Acceleration AX with gravity
    print("\tAY: %.3f"%ay)  # Acceleration AY with gravity
    print("\tAZ: %.3f"%az)  # Acceleration AZ with gravity

def on_gyroscope(gx, gy, gz):
    """Called when gyroscope data is received"""
    print("\tGX: %.3f"%gx)  # Angular velocity GX
    print("\tGY: %.3f"%gy)  # Angular velocity GY
    print("\tGZ: %.3f"%gz)  # Angular velocity GZ

def on_magnetometer(cx, cy, cz, magnitude):
    """Called when magnetometer data is received"""
    print("\tCX: %.3f"%cx);  # Magnetic field data CX
    print("\tCY: %.3f"%cy);  # Magnetic field data CY
    print("\tCZ: %.3f"%cz);  # Magnetic field data CZ

def on_environmental(temperature, air_pressure, height):
    """Called when environmental data is received"""
    print("\ttemperature: %.2f"%temperature)  # temperature
    print("\tairPressure: %.3f"%air_pressure);  # air pressure
    print("\theight: %.3f"%height);  # high

def on_quaternion(w, x, y, z):
    """Called when quaternion data is received"""
    print("\tw: %.3f"%w);  # Quaternion w
    print("\tx: %.3f"%x);  # Quaternion x
    print("\ty: %.3f"%y);  # Quaternion y
    print("\tz: %.3f"%z);  # Quaternion z

def on_euler_angles(roll, pitch, yaw):
    """Called when Euler angle data is received"""
    print("\tangleX: %.3f"%roll);   # Euler angle x (roll)
    print("\tangleY: %.3f"%pitch);  # Euler angle y (pitch)
    print("\tangleZ: %.3f"%yaw);    # Euler angle z (yaw)

def on_unknown_command(command_id):
    """Called when unknown command ID is received"""
    print(f"Error! Command ID not defined: 0x{command_id:02X}.")

# Create callbacks dictionary for the IMU parser
imu_callbacks = {
    'packet_received': on_packet_received,
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

def read_data():
    """
    Main function: initializes IMU sensor and continuously reads data
    """
    print("------------demo start--------------")

    # Parameter settings
    isCompassOn = 0 #Whether to use magnetic field fusion 0: Not used 1: Used
    barometerFilter = 2
    Cmd_ReportTag = 0x02 # Feature subscription tag
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
    Cmd_PackAndTx(params, len(params), ser.write) # Send commands to sensors
    time.sleep(0.2)

    # 2.Wake up sensor
    Cmd_PackAndTx([0x03], 1, ser.write)
    time.sleep(0.2)

    # 3.Enable proactive reporting
    Cmd_PackAndTx([0x19], 1, ser.write)

    # Loop to receive data and process it
    while True:
        data = ser.read(1) # read 1 bytes
        if len(data) > 0: # if data is not empty
            Cmd_GetPkt(data[0], callbacks=imu_callbacks)

# Start reading data
read_data()