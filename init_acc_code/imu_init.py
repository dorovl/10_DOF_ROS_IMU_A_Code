import serial
import time
import math
import numpy as np
from array import array

# Set the correct serial port parameters------------------------
ser_port = "COM18"     #This needs to be replaced with the corresponding serial port number. For Windows systems, it is written as COMx. If it is Linux, it needs to be adjusted according to the system used, such as /dev/ttyUSBx or /dev/ttySx.
ser_baudrate = 115200 # 串Port baud rate

ser_timeout = 2 # Serial port operation timeout time
# Open the serial port
ser = serial.Serial(ser_port, ser_baudrate, timeout=ser_timeout)

def Cmd_RxUnpack(buf, DLen):
    scaleAccel       = 0.00478515625
    scaleQuat        = 0.000030517578125
    scaleAngle       = 0.0054931640625
    scaleAngleSpeed  = 0.06103515625
    scaleMag         = 0.15106201171875
    scaleTemperature = 0.01
    scaleAirPressure = 0.0002384185791
    scaleHeight      = 0.0010728836

    # Handle calibration status messages
    if buf[0] == 0x17:
        global faces_collected
        if DLen >= 2:
            status = buf[1]
            if status == 0xFF:
                print("Calibration finished!")
            elif status >= 0x01 and status <= 0x06:      
                faces_collected = status
                if faces_collected != 6:
                    print(f"Calibration progress: Face {faces_collected}/6 collected. Change the face please!")
                else:
                    print(f"Calibration progress: Face {faces_collected}/6 collected.")
            else:
                print(f"Calibration status: 0x{status:02X}")
        return
    
    # Handle configuration acknowledgments
    if buf[0] == 0x12:
        print("Configuration set successfully")
        return
    
    if buf[0] == 0x03:
        print("Sensor woken up")
        return
    
    if buf[0] == 0x18:
        print("Proactive reporting disabled")
        return
    
    if buf[0] == 0x33:
        print("Range configuration set")
        return

    #print("rev data:",buf)
    if buf[0] == 0x11:
        ctl = (buf[2] << 8) | buf[1]
        print("\n subscribe tag: 0x%04x"%ctl)
        print(" ms: ", ((buf[6]<<24) | (buf[5]<<16) | (buf[4]<<8) | (buf[3]<<0)))

        L =7 # Starting from the 7th byte, the remaining data is parsed according to the subscription identification tag.
        if ((ctl & 0x0001) != 0):  
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
            # print("\taX: %.3f"%tmpX); # Acceleration ax without gravity
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
            # print("\taY: %.3f"%tmpY); # Acceleration ay without gravity
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\taZ: %.3f"%tmpZ); #Acceleration az without gravity
                    
        if ((ctl & 0x0002) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\tAX: %.3f"%tmpX) # Acceleration AX with gravity
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\tAY: %.3f"%tmpY) # Acceleration AY gravity
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            # print("\tAZ: %.3f"%tmpZ) # Acceleration AZ gravity
            tmpAbs = np.sqrt(tmpX*tmpX + tmpY*tmpY + tmpZ*tmpZ)
            print("\tAbs: %.3f"%tmpAbs) # Acceleration module

        if ((ctl & 0x0004) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
            # print("\tGX: %.3f"%tmpX) # Angular velocity GX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
            # print("\tGY: %.3f"%tmpY) # Angular velocity GY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
            # print("\tGZ: %.3f"%tmpZ) # Angular velocity GZ
        
        if ((ctl & 0x0008) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            print("\tCX: %.3f"%tmpX); # Magnetic field data CX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            print("\tCY: %.3f"%tmpY); # Magnetic field data CY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            print("\tCZ: %.3f"%tmpZ); # Magnetic field data CZ
            tmpAbs = math.sqrt(math.pow(tmpX,2) + math.pow(tmpY,2) + math.pow(tmpZ,2)); 
            print("\tCAbs: %.3f"%tmpAbs); # Absolute value of 3-axis composite
        
        if ((ctl & 0x0010) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleTemperature; L += 2
            # print("\ttemperature: %.2f"%tmpX) # temperature

            tmpU32 = np.uint32(((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L])))
            if ((tmpU32 & 0x800000) == 0x800000): # If the highest bit of the 24-digit number is 1, the value is a negative number and needs to be converted to a 32-bit negative number, just add ff directly.
                tmpU32 = (tmpU32 | 0xff000000)      
            tmpY = np.int32(tmpU32) * scaleAirPressure; L += 3
            # print("\tairPressure: %.3f"%tmpY); # air pressure

            tmpU32 = np.uint32((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L]))
            if ((tmpU32 & 0x800000) == 0x800000): # If the highest bit of the 24-digit number is 1, the value is a negative number and needs to be converted to a 32-bit negative number, just add ff directly.
                tmpU32 = (tmpU32 | 0xff000000)
            tmpZ = np.int32(tmpU32) * scaleHeight; L += 3 
            # print("\theight: %.3f"%tmpZ); # high

        if ((ctl & 0x0020) != 0):
            tmpAbs = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # print("\tw: %.3f"%tmpAbs); # Quaternions w
            tmpX =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # print("\tx: %.3f"%tmpX); # Quaternions x
            tmpY =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # print("\ty: %.3f"%tmpY); # Quaternions y
            tmpZ =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            # print("\tz: %.3f"%tmpZ); # Quaternions z

        if ((ctl & 0x0040) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            # print("\tangleX: %.3f"%tmpX); # Euler angles x 
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            # print("\tangleY: %.3f"%tmpY); # Euler angles y 
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            # print("\tangleZ: %.3f"%tmpZ); # Euler angles z
            
    elif buf[0] == 0x34:
        print("\taccelRange: %.3f"%buf[1]); # accelRange 
        print("\tgyroRange: %.3f"%buf[2]); # gyroRange 
    else:
        print("------command ID not defined: 0x%02X"%buf[0])

CmdPacket_Begin = 0x49   # Start code
CmdPacket_End = 0x4D     # end code
CmdPacketMaxDatSizeRx = 73  # The maximum length of the data body of the data packet sent by the module

CS = 0  # Checksum
i = 0
RxIndex = 0

buf = bytearray(5 + CmdPacketMaxDatSizeRx) # Receive packet cache
cmdLen = 0 # length

def Cmd_GetPkt(byte):
    global CS, i, RxIndex, buf, cmdLen
    CS += byte # Calculate the check code while receiving the data. The check code is the sum of the data from the beginning of the address code (including the address code) to before the check code.
    if RxIndex == 0: # Start code
        if byte == CmdPacket_Begin:
            i = 0
            buf[i] = CmdPacket_Begin
            i += 1
            CS = 0 # Calculate the check code starting from the next byte
            RxIndex = 1
    elif RxIndex == 1: # The address code of the data body
        buf[i] = byte
        i += 1
        if byte == 255: # 255 is the broadcast address，module as slave，Its address cannot appear 255
            RxIndex = 0
        else:
            RxIndex += 1
    elif RxIndex == 2: # The length of the data body
        buf[i] = byte
        i += 1
        if byte > CmdPacketMaxDatSizeRx or byte == 0:  # Invalid length
            RxIndex = 0
        else:
            RxIndex += 1
            cmdLen = byte
    elif RxIndex == 3: # Get the data of the data body
        buf[i] = byte
        i += 1
        if i >= cmdLen + 3: # Data body has been received
            RxIndex += 1
    elif RxIndex == 4: # Compare verification code
        CS -= byte
        if (CS&0xFF) == byte: # Verification is correct
            buf[i] = byte
            i += 1
            RxIndex += 1
        else: # Verification failed
            RxIndex = 0
    elif RxIndex == 5: # end code
        RxIndex = 0
        if byte == CmdPacket_End: # 捕Get the complete package
            buf[i] = byte
            i += 1
            hex_string = " ".join(f"{b:02X}" for b in buf[0:i])
            print(f"U-Rx[Len={i}]:{hex_string}")
            Cmd_RxUnpack(buf[3:i-2], i-5) # Process the data body of the packet
            return 1
    else:
        RxIndex = 0
    return 0

def Cmd_PackAndTx(pDat, DLen):
    if DLen == 0 or DLen > 19:
        return -1  # Illegal parameters

    # Build a send packet cache, including the 50-byte preamble
    buf = bytearray([0x00]*46) + bytearray([0x00, 0xff, 0x00, 0xff,  0x49, 0xFF, DLen]) + bytearray(pDat[:DLen])

    # Calculate the checksum, starting from the address code to the end of the data body
    CS = sum(buf[51:51+DLen+2]) & 0xFF  # Take the lower 8 bits
    buf.append(CS)
    buf.append(0x4D)  # Add closing code

    # Send data
    ser.write(buf)
    return 0

def handle_response():
    while True:
        data = ser.read(1)
        if len(data) > 0:
            if Cmd_GetPkt(data[0]) == 1:
                break

def read_data():
    global faces_collected
    faces_collected = 0
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
    params[4] = ((barometerFilter&3)<<1) | (isCompassOn&1);   
    params[5] = 60      #The transmission frame rate of data actively reported [value 0-250HZ], 0 means 0.5HZ
    params[6] = 1       #Gyroscope filter coefficient [value 0-2], the larger the value, the more stable it is but the worse the real-time performance.
    params[7] = 3       #Accelerometer filter coefficient [value 0-4], the larger the value, the more stable it is but the worse the real-time performance.
    params[8] = 5       #Magnetometer filter coefficient [value 0-9], the larger the value, the more stable it is but the worse the real-time performance.
    params[9] = Cmd_ReportTag&0xff
    params[10] = (Cmd_ReportTag>>8)&0xff    
    Cmd_PackAndTx(params, len(params)) # Send commands to sensors
    handle_response()

    # 2.Wake up sensor
    Cmd_PackAndTx([0x03], 1)
    handle_response()

    # 3.Disable proactive reporting
    Cmd_PackAndTx([0x18], 1)
    handle_response()

    # 4.Set the range of accelerometer and gyroscope
    # AccRange range 0=2g 1=4g 2=8g 3=16g
    # GyroRange range 0=256 1=512 2=1024 3=2048
    Cmd_PackAndTx([0x33,0x00,0x00], 3)
    handle_response()

    # 5. Start accelerometer calibration process
    Cmd_PackAndTx([0x17,0x00], 2)
    while faces_collected < 6:
        data = ser.read(1)
        if len(data) > 0:
            Cmd_GetPkt(data[0])
    
    # 6. Committing accelerometer calibration
    Cmd_PackAndTx([0x17,0xff], 2)
    handle_response()

    # 7. Reading the range of accelerometer and gyroscope
    # AccRange range 0=2g 1=4g 2=8g 3=16g
    # GyroRange range 0=256 1=512 2=1024 3=2048
    Cmd_PackAndTx([0x34], 1)
    handle_response()

    # 8. Read one time
    print("Place sensor flat and keep it completely still...")
    for i in range(10, 0, -1):
        print(f"\rStarting verification in {i} seconds... ", end='', flush=True)
        time.sleep(1)
    print("\rTaking measurement...                       ", flush=True)
    Cmd_PackAndTx([0x11], 1) 
    handle_response()
    print("\nCalibration complete! The acceleration reading should be close to 9.81 m/s²")
            
# Start reading data
read_data()