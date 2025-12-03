import serial
import time

# Import the reusable IMU parser library
from imu_parser import Cmd_GetPkt, Cmd_PackAndTx

# Set the correct serial port parameters------------------------
ser_port = "COM18"     #This needs to be replaced with the corresponding serial port number. For Windows systems, it is written as COMx. If it is Linux, it needs to be adjusted according to the system used, such as /dev/ttyUSBx or /dev/ttySx.
ser_baudrate = 115200 # 串Port baud rate
ser_timeout = 2 # Serial port operation timeout time

# ========== IMU Data Callbacks ==========

def on_packet_received(hex_string, length):
    """Called when a complete packet is received"""
    print(f"U-Rx[Len={length}]:{hex_string}")

def on_config_ack():
    """Called when configuration acknowledgment is received (0x12)"""
    print("Configuration set successfully.")

def on_wakeup_ack():
    """Called when sensor wake-up acknowledgment is received (0x03)"""
    print("Sensor woken up.")

def on_reporting_disabled_ack():
    """Called when proactive reporting disabled acknowledgment is received (0x18)"""
    print("Proactive reporting disabled.")

def on_mag_calibration_started_ack():
    """Called when magnetometer calibration started acknowledgment is received (0x32)"""
    print("Magnetometer calibration started.")

def on_mag_calibration_saved_ack():
    """Called when magnetometer calibration saved acknowledgment is received (0x04)"""
    print("Magnetometer calibration saved!")

def on_z_axis_reset_ack():
    """Called when Z-axis angle reset acknowledgment is received (0x05)"""
    print("Z-axis angle reset to zero.")

def on_packet_header(tag, timestamp_ms):
    """Called when packet header is parsed"""
    print("\n subscribe tag: 0x%04x"%tag)
    print(" ms: ", timestamp_ms)

def on_magnetometer(cx, cy, cz, magnitude):
    """Called when magnetometer data is received"""
    print("\tCX: %.3f"%cx); # Magnetic field data CX
    print("\tCY: %.3f"%cy); # Magnetic field data CY
    print("\tCZ: %.3f"%cz); # Magnetic field data CZ
    print("\tCAbs: %.3f"%magnitude); # Absolute value of 3-axis composite

def on_unknown_command(command_id):
    """Called when unknown command ID is received"""
    print(f"Error! Command ID not defined: 0x{command_id:02X}.")

# Create callbacks dictionary for the IMU parser
imu_callbacks = {
    'packet_received': on_packet_received,
    'config_ack': on_config_ack,
    'wakeup_ack': on_wakeup_ack,
    'reporting_disabled_ack': on_reporting_disabled_ack,
    'mag_calibration_started_ack': on_mag_calibration_started_ack,
    'mag_calibration_saved_ack': on_mag_calibration_saved_ack,
    'z_axis_reset_ack': on_z_axis_reset_ack,
    'packet_header': on_packet_header,
    'magnetometer': on_magnetometer,
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
    """Main magnetometer calibration function"""
    with serial.Serial(ser_port, ser_baudrate, timeout=ser_timeout) as ser:
        print("=== Starting Magnetometer Calibration ===")
        print("Rotate the sensor randomly in all 3D directions to sample the complete magnetic field sphere.")
        print("Flip, tilt, and rotate in every possible orientation for best calibration results.")
        print()
        input("Press Enter to start...")
        print("Rotate sensor now!")

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
        Cmd_PackAndTx(params, len(params), ser.write) # Send commands to sensors
        handle_response(ser)

        # 2.Wake up sensor
        Cmd_PackAndTx([0x03], 1, ser.write)
        handle_response(ser)

        # 3.Disable proactive reporting (keep it off during calibration)
        Cmd_PackAndTx([0x18], 1, ser.write)
        handle_response(ser)

        # 4. Start magnetometer calibration
        Cmd_PackAndTx([0x32], 1, ser.write)
        handle_response(ser)
        for i in range(30, 0, -1):
            print(f"\r{i} seconds remaining... ", end='', flush=True)
            time.sleep(1)

        # 5. Finish magnetometer calibration
        Cmd_PackAndTx([0x04], 1, ser.write)
        handle_response(ser)

        # 6. Z axis angle reset to zero
        Cmd_PackAndTx([0x05], 1, ser.write)
        handle_response(ser)

        # Read final data
        Cmd_PackAndTx([0x11], 1, ser.write)
        handle_response(ser)

        print("Magnetometer Calibration Complete!")

if __name__ == "__main__":
    main()