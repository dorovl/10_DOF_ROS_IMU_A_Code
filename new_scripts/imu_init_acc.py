import serial
import time

# Import the reusable IMU parser library
from imu_parser import Cmd_GetPkt, Cmd_PackAndTx

# Set the correct serial port parameters------------------------
ser_port = "COM18"     #This needs to be replaced with the corresponding serial port number. For Windows systems, it is written as COMx. If it is Linux, it needs to be adjusted according to the system used, such as /dev/ttyUSBx or /dev/ttySx.
ser_baudrate = 115200 # 串Port baud rate
ser_timeout = 2 # Serial port operation timeout time

# Global variable to track calibration progress
faces_collected = 0

# ========== IMU Data Callbacks ==========

def on_packet_received(hex_string, length):
    """Called when a complete packet is received"""
    print(f"U-Rx[Len={length}]:{hex_string}")

def on_calibration_status(status):
    """Called when accelerometer calibration status is received (0x17)"""
    global faces_collected
    if status == 0xFF:
        print("Calibration saved!")
    elif status >= 0x01 and status <= 0x06:
        faces_collected = status
        if faces_collected != 6:
            print(f"Calibration progress: Face {faces_collected}/6 collected. Change the face please!")
        else:
            print(f"Calibration progress: Face {faces_collected}/6 collected.")
    else:
        print(f"Calibration status: 0x{status:02X}.")

def on_config_ack():
    """Called when configuration acknowledgment is received (0x12)"""
    print("Configuration set successfully.")

def on_wakeup_ack():
    """Called when sensor wake-up acknowledgment is received (0x03)"""
    print("Sensor woken up.")

def on_reporting_disabled_ack():
    """Called when proactive reporting disabled acknowledgment is received (0x18)"""
    print("Proactive reporting disabled.")

def on_range_config_ack():
    """Called when range configuration acknowledgment is received (0x33)"""
    print("Range configuration set.")

def on_packet_header(tag, timestamp_ms):
    """Called when packet header is parsed"""
    print("\n subscribe tag: 0x%04x"%tag)
    print(" ms: ", timestamp_ms)

def on_accel_with_gravity(ax, ay, az, magnitude):
    """Called when acceleration data with gravity is received"""
    # Acceleration AX with gravity
    # Acceleration AY with gravity
    # Acceleration AZ with gravity
    print("\tAbs: %.3f"%magnitude) # Acceleration module

def on_range_response(accel_range, gyro_range):
    """Called when range configuration response is received (0x34)"""
    print("\taccelRange: %.3f"%accel_range); # accelRange
    print("\tgyroRange: %.3f"%gyro_range); # gyroRange

def on_unknown_command(command_id):
    """Called when unknown command ID is received"""
    print(f"Error! Command ID not defined: 0x{command_id:02X}.")

# Create callbacks dictionary for the IMU parser
imu_callbacks = {
    'packet_received': on_packet_received,
    'calibration_status': on_calibration_status,
    'config_ack': on_config_ack,
    'wakeup_ack': on_wakeup_ack,
    'reporting_disabled_ack': on_reporting_disabled_ack,
    'range_config_ack': on_range_config_ack,
    'packet_header': on_packet_header,
    'accel_with_gravity': on_accel_with_gravity,
    'range_response': on_range_response,
    'unknown_command': on_unknown_command,
}

def handle_response(ser):
    """Wait for and process a complete response packet"""
    while True:
        data = ser.read(1)
        if len(data) > 0:
            if Cmd_GetPkt(data[0], callbacks=imu_callbacks) == 1:
                break

def main():
    """Main accelerometer calibration function"""
    global faces_collected
    faces_collected = 0

    with serial.Serial(ser_port, ser_baudrate, timeout=ser_timeout) as ser:
        print("=== Starting Accelerometer Calibration ===")
        print("Place the sensor on first face and keep it still. Once prompted for, change the face.")
        print("Note: The LED will pause briefly when each face is collected.")

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
        handle_response(ser)

        # 2.Wake up sensor
        Cmd_PackAndTx([0x03], 1, ser.write)
        handle_response(ser)

        # 3.Disable proactive reporting
        Cmd_PackAndTx([0x18], 1, ser.write)
        handle_response(ser)

        # 4.Set the range of accelerometer and gyroscope
        # AccRange range 0=2g 1=4g 2=8g 3=16g
        # GyroRange range 0=256 1=512 2=1024 3=2048
        Cmd_PackAndTx([0x33,0x00,0x00], 3, ser.write)
        handle_response(ser)

        # 5. Start accelerometer calibration process
        Cmd_PackAndTx([0x17,0x00], 2, ser.write)
        while faces_collected < 6:
            data = ser.read(1)
            if len(data) > 0:
                Cmd_GetPkt(data[0], callbacks=imu_callbacks)

        # 6. Saving accelerometer calibration
        Cmd_PackAndTx([0x17,0xff], 2, ser.write)
        handle_response(ser)

        # 7. Reading the range of accelerometer and gyroscope
        # AccRange range 0=2g 1=4g 2=8g 3=16g
        # GyroRange range 0=256 1=512 2=1024 3=2048
        Cmd_PackAndTx([0x34], 1, ser.write)
        handle_response(ser)

        # 8. Read one time
        print("Place sensor flat and keep it completely still...")
        for i in range(10, 0, -1):
            print(f"\rStarting verification in {i} seconds... ", end='', flush=True)
            time.sleep(1)
        print("\rTaking measurement...                       ", flush=True)
        Cmd_PackAndTx([0x11], 1, ser.write)
        handle_response(ser)
        print("\nCalibration complete! The acceleration reading should be close to 9.81 m/s².")

if __name__ == "__main__":
    main()