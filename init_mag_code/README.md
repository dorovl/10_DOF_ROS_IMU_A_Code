### Depends on installation
    pip install pyserial numpy 
### How to use
Line 7 in the code ser_port = "COM13"  #The ser_port needs to be replaced with the orresponding serial port number. 

For Windows systems, it is written as COMx. If it is Linux, it needs to be adjusted according to the system used, such as /dev/ttyUSBx or /dev/ttySx.

Within 10 seconds, rotate the Z-axis of the module perpendicular to the horizontal plane and the Z-axis balanced on the horizontal plane and rotate more than once in the horizontal plane

Initialization imu:

    python3 imu_init.py