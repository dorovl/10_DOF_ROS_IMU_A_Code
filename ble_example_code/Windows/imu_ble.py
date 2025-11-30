import asyncio
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from array import array
import numpy as np

# Characteristic UUID of the device
# par_notification_characteristic="0000ae02-0000-1000-8000-00805f9b34fb"
par_notification_characteristic=0x0007
# Characteristic UUID of the device (with write attribute Write)
# par_write_characteristic="0000ae01-0000-1000-8000-00805f9b34fb"
par_write_characteristic=0x0005

par_device_addr="70:53:B2:02:20:02" # MAC address of the device You need to fill in the mac address of the device here

# Listening callback function, here is the print message
def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    #print("rev data:",data)
    parse_imu(data)

def parse_imu(buf):
    scaleAccel       = 0.00478515625      # acceleration [-16g~+16g]    9.8*16/32768
    scaleQuat        = 0.000030517578125  # Quaternions [-1~+1]         1/32768
    scaleAngle       = 0.0054931640625    # angle   [-180~+180]     180/32768
    scaleAngleSpeed  = 0.06103515625      # Angular velocity [-2000~+2000]    2000/32768
    scaleMag         = 0.15106201171875   # magnetic field [-4950~+4950]   4950/32768
    scaleTemperature = 0.01               # temperature
    scaleAirPressure = 0.0002384185791    # air pressure [-2000~+2000]    2000/8388608
    scaleHeight      = 0.0010728836       # high [-9000~+9000]    9000/8388608

    imu_dat = array('f',[0.0 for i in range(0,34)])

    if buf[0] == 0x11:
        ctl = (buf[2] << 8) | buf[1]
        print("\n subscribe tag: 0x%04x"%ctl)
        print(" ms: ", ((buf[6]<<24) | (buf[5]<<16) | (buf[4]<<8) | (buf[3]<<0)))

        L =7 # Starting from the 7th byte, the remaining data is parsed according to the subscription identification tag.
        if ((ctl & 0x0001) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
            print("\taX: %.3f"%tmpX); # Acceleration ax without gravity
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
            print("\taY: %.3f"%tmpY); # Acceleration ay without gravity
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            print("\taZ: %.3f"%tmpZ); #Acceleration az without gravity

            imu_dat[0] = float(tmpX)
            imu_dat[1] = float(tmpY)
            imu_dat[2] = float(tmpZ)
        
        if ((ctl & 0x0002) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            print("\tAX: %.3f"%tmpX) # Acceleration AX with gravity
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            print("\tAY: %.3f"%tmpY) # Acceleration AY gravity
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            print("\tAZ: %.3f"%tmpZ) # Acceleration AZ gravity

            imu_dat[3] = float(tmpX)
            imu_dat[4] = float(tmpY)
            imu_dat[5] = float(tmpZ)

        if ((ctl & 0x0004) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
            print("\tGX: %.3f"%tmpX) # Angular velocity GX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
            print("\tGY: %.3f"%tmpY) # Angular velocity GY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
            print("\tGZ: %.3f"%tmpZ) # Angular velocity GZ

            imu_dat[6] = float(tmpX)
            imu_dat[7] = float(tmpY)
            imu_dat[8] = float(tmpZ)
        
        if ((ctl & 0x0008) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            print("\tCX: %.3f"%tmpX); # Magnetic field data CX
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            print("\tCY: %.3f"%tmpY); # Magnetic field data CY
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            print("\tCZ: %.3f"%tmpZ); # Magnetic field data CZ

            imu_dat[9] = float(tmpX)
            imu_dat[10] = float(tmpY)
            imu_dat[11] = float(tmpZ)
        
        if ((ctl & 0x0010) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleTemperature; L += 2
            print("\ttemperature: %.2f"%tmpX) # temperature

            tmpU32 = np.uint32(((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L])))
            if ((tmpU32 & 0x800000) == 0x800000): # If the highest bit of the 24-digit number is 1, the value is a negative number and needs to be converted to a 32-bit negative number, just add ff directly.
                tmpU32 = (tmpU32 | 0xff000000)      
            tmpY = np.int32(tmpU32) * scaleAirPressure; L += 3
            print("\tairPressure: %.3f"%tmpY); # air pressure

            tmpU32 = np.uint32((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L]))
            if ((tmpU32 & 0x800000) == 0x800000): # If the highest bit of the 24-digit number is 1, the value is a negative number and needs to be converted to a 32-bit negative number, just add ff directly.
                tmpU32 = (tmpU32 | 0xff000000)
            tmpZ = np.int32(tmpU32) * scaleHeight; L += 3 
            print("\theight: %.3f"%tmpZ); # high

            imu_dat[12] = float(tmpX)
            imu_dat[13] = float(tmpY)
            imu_dat[14] = float(tmpZ)

        if ((ctl & 0x0020) != 0):
            tmpAbs = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            print("\tw: %.3f"%tmpAbs); # Quaternions w
            tmpX =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            print("\tx: %.3f"%tmpX); # Quaternions x
            tmpY =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            print("\ty: %.3f"%tmpY); # Quaternions y
            tmpZ =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            print("\tz: %.3f"%tmpZ); # Quaternions z

            imu_dat[15] = float(tmpAbs)
            imu_dat[16] = float(tmpX)
            imu_dat[17] = float(tmpY)
            imu_dat[18] = float(tmpZ)

        if ((ctl & 0x0040) != 0):
            tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            print("\tangleX: %.3f"%tmpX); # Euler angles x 
            tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            print("\tangleY: %.3f"%tmpY); # Euler angles y 
            tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
            print("\tangleZ: %.3f"%tmpZ); # Euler angles z

            imu_dat[19] = float(tmpX)
            imu_dat[20] = float(tmpY)
            imu_dat[21] = float(tmpZ)

    else:
        print("[error] data head not define")

async def main():
    print("starting scan...")

    #Find devices based on MAC address
    device = await BleakScanner.find_device_by_address(
        par_device_addr, cb=dict(use_bdaddr=False)  #use_bdaddr determines whether it is MOC system统
    )
    if device is None:
        print("could not find device with address '%s'", par_device_addr)
        return

    #event definition
    disconnected_event = asyncio.Event()

    #Disconnect event callback
    def disconnected_callback(client):
        print("Disconnected callback called!")
        disconnected_event.set()

    print("connecting to device...")
    async with BleakClient(device,disconnected_callback=disconnected_callback) as client:
        print("Connected")
        await client.start_notify(par_notification_characteristic, notification_handler)
   
        # stay connected 0x29
        wakestr=bytes([0x29])
        await client.write_gatt_char(par_write_characteristic, wakestr)
        await asyncio.sleep(0.2)
        print("------------------------------------------------")
        # Try to use Bluetooth high-speed communication features 0x46
        fast=bytes([0x46])
        await client.write_gatt_char(par_write_characteristic, fast)
        await asyncio.sleep(0.2)

        # Parameter settings
        isCompassOn = 0 #Whether to use magnetic field fusion 0: Not used 1: Used
        barometerFilter = 2
        Cmd_ReportTag = 0x7F # Feature subscription tag
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
        await client.write_gatt_char(par_write_characteristic, params)
        await asyncio.sleep(0.2)

        notes=bytes([0x19])
        await client.write_gatt_char(par_write_characteristic, notes)
           
        # Add a loop so that the program does not exit while receiving data
        while not disconnected_event.is_set():
            await asyncio.sleep(1.0)
            
        #await disconnected_event.wait() #Sleep until device disconnects, with delay. Here is the listening device until disconnected
        #await client.stop_notify(par_notification_characteristic)

asyncio.run(main())