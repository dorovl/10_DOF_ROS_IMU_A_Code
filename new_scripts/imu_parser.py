# imu_parser.py - Complete IMU serial protocol parser library

import numpy as np
import math

# ========== Protocol Constants ==========
CmdPacket_Begin = 0x49   # Start code
CmdPacket_End = 0x4D     # end code
CmdPacketMaxDatSizeRx = 73  # The maximum length of the data body of the data packet sent by the module

# ========== Packet State Machine Globals ==========
# These maintain state for the byte-by-byte packet parser
_CS = 0  # Checksum
_i = 0
_RxIndex = 0
_buf = bytearray(5 + CmdPacketMaxDatSizeRx) # Receive packet cache
_cmdLen = 0 # length

# Scale factors for converting raw sensor data to physical units
scaleAccel       = 0.00478515625      # Acceleration scale factor
scaleQuat        = 0.000030517578125  # Quaternion scale factor
scaleAngle       = 0.0054931640625    # Angle scale factor (degrees)
scaleAngleSpeed  = 0.06103515625      # Angular velocity scale factor (deg/s)
scaleMag         = 0.15106201171875   # Magnetometer scale factor
scaleTemperature = 0.01               # Temperature scale factor (°C)
scaleAirPressure = 0.0002384185791    # Air pressure scale factor
scaleHeight      = 0.0010728836       # Height scale factor

def Cmd_RxUnpack(buf, DLen, callbacks=None):
    """
    Parse IMU data packet and invoke callbacks for different data types.

    Args:
        buf: Data buffer containing the packet data body
        DLen: Data length
        callbacks: Dictionary of callback functions:
            - 'packet_header': callback(tag, timestamp_ms)
            - 'accel_no_gravity': callback(ax, ay, az)
            - 'accel_with_gravity': callback(ax, ay, az, magnitude)
            - 'gyroscope': callback(gx, gy, gz)
            - 'magnetometer': callback(cx, cy, cz, magnitude)
            - 'environmental': callback(temperature, air_pressure, height)
            - 'quaternion': callback(w, x, y, z)
            - 'euler_angles': callback(roll, pitch, yaw)
            - 'calibration_status': callback(status)
            - 'config_ack': callback()
            - 'wakeup_ack': callback()
            - 'reporting_disabled_ack': callback()
            - 'reporting_enabled_ack': callback()
            - 'range_config_ack': callback()
            - 'mag_calibration_started_ack': callback()
            - 'mag_calibration_saved_ack': callback()
            - 'z_axis_reset_ack': callback()
            - 'range_response': callback(accel_range, gyro_range)
            - 'unknown_command': callback(command_id)
    """
    if callbacks is None:
        callbacks = {}

    # Handle accelerometer calibration status messages (0x17)
    if buf[0] == 0x17:
        if DLen >= 2:
            status = buf[1]
            if 'calibration_status' in callbacks:
                callbacks['calibration_status'](status)
        return

    # Handle configuration acknowledgment (0x12)
    if buf[0] == 0x12:
        if 'config_ack' in callbacks:
            callbacks['config_ack']()
        return

    # Handle sensor wake-up acknowledgment (0x03)
    if buf[0] == 0x03:
        if 'wakeup_ack' in callbacks:
            callbacks['wakeup_ack']()
        return

    # Handle proactive reporting disabled acknowledgment (0x18)
    if buf[0] == 0x18:
        if 'reporting_disabled_ack' in callbacks:
            callbacks['reporting_disabled_ack']()
        return

    # Handle proactive reporting enabled acknowledgment (0x19)
    if buf[0] == 0x19:
        if 'reporting_enabled_ack' in callbacks:
            callbacks['reporting_enabled_ack']()
        return

    # Handle range configuration acknowledgment (0x33)
    if buf[0] == 0x33:
        if 'range_config_ack' in callbacks:
            callbacks['range_config_ack']()
        return

    # Handle magnetometer calibration started acknowledgment (0x32)
    if buf[0] == 0x32:
        if 'mag_calibration_started_ack' in callbacks:
            callbacks['mag_calibration_started_ack']()
        return

    # Handle magnetometer calibration saved acknowledgment (0x04)
    if buf[0] == 0x04:
        if 'mag_calibration_saved_ack' in callbacks:
            callbacks['mag_calibration_saved_ack']()
        return

    # Handle Z-axis angle reset acknowledgment (0x05)
    if buf[0] == 0x05:
        if 'z_axis_reset_ack' in callbacks:
            callbacks['z_axis_reset_ack']()
        return

    # Handle range configuration response (0x34)
    if buf[0] == 0x34:
        if DLen >= 2:
            accel_range = buf[1]  # AccRange range 0=2g 1=4g 2=8g 3=16g
            gyro_range = buf[2]   # GyroRange range 0=256 1=512 2=1024 3=2048
            if 'range_response' in callbacks:
                callbacks['range_response'](accel_range, gyro_range)
        return

    # Check if this is a valid IMU data packet (starts with 0x11)
    if buf[0] == 0x11:
        # Parse packet header
        ctl = (buf[2] << 8) | buf[1]  # Control/subscription tag
        timestamp_ms = ((buf[6]<<24) | (buf[5]<<16) | (buf[4]<<8) | (buf[3]<<0))  # Timestamp in milliseconds

        # Notify about packet header
        if 'packet_header' in callbacks:
            callbacks['packet_header'](ctl, timestamp_ms)

        L = 7  # Starting from the 7th byte, the remaining data is parsed according to the subscription identification tag.

        # Parse acceleration data without gravity (0x0001)
        if ((ctl & 0x0001) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # Acceleration ax without gravity
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # Acceleration ay without gravity
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # Acceleration az without gravity

            if 'accel_no_gravity' in callbacks:
                callbacks['accel_no_gravity'](tmpX, tmpY, tmpZ)

        # Parse acceleration data with gravity (0x0002)
        if ((ctl & 0x0002) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # Acceleration AX with gravity
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # Acceleration AY with gravity
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # Acceleration AZ with gravity

            # Calculate acceleration magnitude (absolute value of 3-axis composite)
            tmpAbs = np.sqrt(tmpX*tmpX + tmpY*tmpY + tmpZ*tmpZ)
            # Acceleration module

            if 'accel_with_gravity' in callbacks:
                callbacks['accel_with_gravity'](tmpX, tmpY, tmpZ, tmpAbs)

        # Parse gyroscope data (0x0004)
        if ((ctl & 0x0004) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
            # Angular velocity GX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
            # Angular velocity GY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
            # Angular velocity GZ

            if 'gyroscope' in callbacks:
                callbacks['gyroscope'](tmpX, tmpY, tmpZ)

        # Parse magnetometer data (0x0008)
        if ((ctl & 0x0008) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            # Magnetic field data CX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            # Magnetic field data CY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            # Magnetic field data CZ

            # Calculate magnetometer magnitude (absolute value of 3-axis composite)
            tmpAbs = math.sqrt(math.pow(tmpX, 2) + math.pow(tmpY, 2) + math.pow(tmpZ, 2))
            # Absolute value of 3-axis composite

            if 'magnetometer' in callbacks:
                callbacks['magnetometer'](tmpX, tmpY, tmpZ, tmpAbs)

        # Parse environmental data: temperature, air pressure, height (0x0010)
        if ((ctl & 0x0010) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleTemperature; L += 2
            # temperature

            # Parse 24-bit air pressure value
            tmpU32 = np.uint32(((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L])))
            # If the highest bit of the 24-digit number is 1, the value is a negative number and needs to be converted to a 32-bit negative number, just add ff directly.
            if ((tmpU32 & 0x800000) == 0x800000):
                tmpU32 = (tmpU32 | 0xff000000)
            tmpY = np.int32(tmpU32) * scaleAirPressure; L += 3
            # air pressure

            # Parse 24-bit height value
            tmpU32 = np.uint32((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L]))
            # If the highest bit of the 24-digit number is 1, the value is a negative number and needs to be converted to a 32-bit negative number, just add ff directly.
            if ((tmpU32 & 0x800000) == 0x800000):
                tmpU32 = (tmpU32 | 0xff000000)
            tmpZ = np.int32(tmpU32) * scaleHeight; L += 3
            # high

            if 'environmental' in callbacks:
                callbacks['environmental'](tmpX, tmpY, tmpZ)

        # Parse quaternion data (0x0020)
        if ((ctl & 0x0020) != 0):
            tmpAbs = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # Quaternion w
            tmpX =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # Quaternion x
            tmpY =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # Quaternion y
            tmpZ =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # Quaternion z

            if 'quaternion' in callbacks:
                callbacks['quaternion'](tmpAbs, tmpX, tmpY, tmpZ)

        # Parse Euler angles (0x0040)
        if ((ctl & 0x0040) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            # Euler angle x (roll)
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            # Euler angle y (pitch)
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            # Euler angle z (yaw)

            if 'euler_angles' in callbacks:
                callbacks['euler_angles'](tmpX, tmpY, tmpZ)

    else:
        # Unknown command ID
        if 'unknown_command' in callbacks:
            callbacks['unknown_command'](buf[0])


def Cmd_GetPkt(byte, callbacks=None):
    """
    State machine for parsing incoming serial data packets byte-by-byte.
    Call this function for each byte received from the serial port.

    Args:
        byte: Single byte received from serial port
        callbacks: Dictionary of callbacks to pass to Cmd_RxUnpack when packet is complete

    Returns:
        1 when a complete packet is received and processed, 0 otherwise
    """
    global _CS, _i, _RxIndex, _buf, _cmdLen

    _CS += byte # Calculate the check code while receiving the data. The check code is the sum of the data from the beginning of the address code (including the address code) to before the check code.

    if _RxIndex == 0: # Start code
        if byte == CmdPacket_Begin:
            _i = 0
            _buf[_i] = CmdPacket_Begin
            _i += 1
            _CS = 0 # Calculate the check code starting from the next byte
            _RxIndex = 1

    elif _RxIndex == 1: # The address code of the data body
        _buf[_i] = byte
        _i += 1
        if byte == 255: # 255 is the broadcast address, module as slave, Its address cannot appear 255
            _RxIndex = 0
        else:
            _RxIndex += 1

    elif _RxIndex == 2: # The length of the data body
        _buf[_i] = byte
        _i += 1
        if byte > CmdPacketMaxDatSizeRx or byte == 0:  # Invalid length
            _RxIndex = 0
        else:
            _RxIndex += 1
            _cmdLen = byte

    elif _RxIndex == 3: # Get the data of the data body
        _buf[_i] = byte
        _i += 1
        if _i >= _cmdLen + 3: # Data body has been received
            _RxIndex += 1

    elif _RxIndex == 4: # Compare verification code
        _CS -= byte
        if (_CS&0xFF) == byte: # Verification is correct
            _buf[_i] = byte
            _i += 1
            _RxIndex += 1
        else: # Verification failed
            _RxIndex = 0

    elif _RxIndex == 5: # end code
        _RxIndex = 0
        if byte == CmdPacket_End: # Get the complete package
            _buf[_i] = byte
            _i += 1
            hex_string = " ".join(f"{b:02X}" for b in _buf[0:_i])

            # Call packet_received callback if provided
            if callbacks and 'packet_received' in callbacks:
                callbacks['packet_received'](hex_string, _i)

            # Process the data body of the packet
            Cmd_RxUnpack(_buf[3:_i-2], _i-5, callbacks=callbacks)
            return 1
    else:
        _RxIndex = 0

    return 0


def Cmd_PackAndTx(pDat, DLen, serial_write_func):
    """
    Build and transmit a command packet to the IMU sensor.

    Args:
        pDat: Data bytes to send
        DLen: Length of data
        serial_write_func: Function to call to write bytes to serial port (e.g., ser.write)

    Returns:
        0 on success, -1 on error
    """
    if DLen == 0 or DLen > 19:
        return -1  # Illegal parameters

    # Build a send packet cache, including the 50-byte preamble
    buf = bytearray([0x00]*46) + bytearray([0x00, 0xff, 0x00, 0xff,  0x49, 0xFF, DLen]) + bytearray(pDat[:DLen])

    # Calculate the checksum, starting from the address code to the end of the data body
    CS = sum(buf[51:51+DLen+2]) & 0xFF  # Take the lower 8 bits
    buf.append(CS)
    buf.append(0x4D)  # Add closing code

    # Send data via provided serial write function
    serial_write_func(buf)
    return 0

def handle_response(ser, imu_callbacks):
    """Wait for and process a complete response packet"""
    while True:
        data = ser.read(1)
        if len(data) > 0:
            if Cmd_GetPkt(data[0], callbacks=imu_callbacks) == 1:
                break