import serial
import time

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *

# Import the reusable IMU parser library
from imu_parser import Cmd_GetPkt, Cmd_PackAndTx, handle_response

# Set the correct serial port parameters------------------------
ser_port = "COM18"     #This needs to be replaced with the corresponding serial port number. For Windows systems, it is written as COMx. If it is Linux, it needs to be adjusted according to the system used, such as /dev/ttyUSBx or /dev/ttySx.
ser_baudrate = 115200 # Port baud rate
ser_timeout = 2 # Serial port operation timeout time

# Global orientation variables
roll = pitch = yaw = 0.0

# Initialize pygame and OpenGL
video_flags = OPENGL|DOUBLEBUF
pygame.init()
screen = pygame.display.set_mode((640,480), video_flags)
pygame.display.set_caption("Press Esc to quit")

def resize(width, height):
    """Setup OpenGL viewport and projection"""
    if height==0:
        height=1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    """Initialize OpenGL rendering settings"""
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def drawText(position, textString):
    """Draw text on the OpenGL screen"""
    font = pygame.font.SysFont ("Courier", 18, True)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def draw():
    """Render the 3D IMU visualization"""
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity()
    glTranslatef(0,0.0,-7.0)

    # Display orientation values as text
    osd_text = "pitch: " + str("{0:.2f}".format(pitch)) + ", roll: " + str("{0:.2f}".format(roll))
    osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(yaw))
    drawText((-2,-2, 2), osd_line)

    # the way I'm holding the IMU board, X and Y axis are switched
    # with respect to the OpenGL coordinate system
    glRotatef(yaw, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
    glRotatef(-1*pitch ,1.0,0.0,0.0)        # Pitch, rotate around x-axis
    glRotatef(-1*roll ,0.0,0.0,1.0)     # Roll,  rotate around z-axis

    # Draw a colored box representing the IMU board
    glBegin(GL_QUADS)
    glColor3f(0.0,1.0,0.0)
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f( 1.0, 0.2, 1.0)

    glColor3f(1.0,0.5,0.0)
    glVertex3f( 1.0,-0.2, 1.0)
    glVertex3f(-1.0,-0.2, 1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f( 1.0,-0.2,-1.0)

    glColor3f(1.0,0.0,0.0)
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0,-0.2, 1.0)
    glVertex3f( 1.0,-0.2, 1.0)

    glColor3f(1.0,1.0,0.0)
    glVertex3f( 1.0,-0.2,-1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)
    glVertex3f( 1.0, 0.2,-1.0)

    glColor3f(0.0,0.0,1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2,-1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f(-1.0,-0.2, 1.0)

    glColor3f(1.0,0.0,1.0)
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f( 1.0,-0.2, 1.0)
    glVertex3f( 1.0,-0.2,-1.0)
    glEnd()

# ========== IMU Data Callbacks ==========
# These callbacks are invoked by the imu_parser when data is received

def on_config_ack():
    """Called when configuration acknowledgment is received (0x12)"""
    print("Configuration set successfully.")

def on_wakeup_ack():
    """Called when sensor wake-up acknowledgment is received (0x03)"""
    print("Sensor woken up.")

def on_reporting_enabled_ack():
    """Called when proactive reporting enabled acknowledgment is received (0x19)"""
    print("Proactive reporting enabled.")

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

def on_euler_angles(roll_val, pitch_val, yaw_val):
    """Called when Euler angle data is received - updates visualization"""
    global roll, pitch, yaw

    print("\tangleX: %.3f"%roll_val);   # Euler angle x (roll)
    print("\tangleY: %.3f"%pitch_val);  # Euler angle y (pitch)
    print("\tangleZ: %.3f"%yaw_val);    # Euler angle z (yaw)

    # Update global orientation variables
    roll = roll_val
    pitch = pitch_val
    yaw = yaw_val

    # Redraw the 3D visualization
    draw()
    pygame.display.flip()

def on_unknown_command(command_id):
    """Called when unknown command ID is received"""
    print(f"Error! Command ID not defined: 0x{command_id:02X}.")

# Create callbacks dictionary for the IMU parser
imu_callbacks = {
    'config_ack': on_config_ack,
    'wakeup_ack': on_wakeup_ack,
    'reporting_enabled_ack' : on_reporting_enabled_ack,
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

def main():
    with serial.Serial(ser_port, ser_baudrate, timeout=ser_timeout) as ser:
        print("------------demo start--------------")

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
        handle_response(ser, imu_callbacks)

        # 2.Wake up sensor
        Cmd_PackAndTx([0x03], 1, ser.write)
        handle_response(ser, imu_callbacks)

        # 3.Enable proactive reporting
        Cmd_PackAndTx([0x19], 1, ser.write)
        handle_response(ser, imu_callbacks)

        resize(640,480)
        init()

        # Loop to receive data and process it
        while True:
            event = pygame.event.poll()
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()  #* quit pygame properly
                break

            data = ser.read(1) # read 1 bytes
            if len(data) > 0: # if data is not empty
                Cmd_GetPkt(data[0], callbacks=imu_callbacks)

if __name__ == "__main__":
    main()