import serial

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

# Global orientation variables (quaternion for rotation)
quat_w = 1.0
quat_x = 0.0
quat_y = 0.0
quat_z = 0.0

# Euler angles for display only
roll = 0.0
pitch = 0.0
yaw = 0.0

# Initialize pygame and OpenGL
video_flags = OPENGL|DOUBLEBUF
pygame.init()
screen = pygame.display.set_mode((1024,768), video_flags)
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

def quat_to_matrix(w, x, y, z):
    """Convert quaternion to 4x4 OpenGL rotation matrix"""
    # Normalize quaternion
    norm = (w*w + x*x + y*y + z*z) ** 0.5
    if norm > 0:
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # Quaternion to rotation matrix conversion
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    
    # OpenGL uses column-major order
    matrix = [
        1-2*(yy+zz), 2*(xy+wz),   2*(xz-wy),   0,
        2*(xy-wz),   1-2*(xx+zz), 2*(yz+wx),   0,
        2*(xz+wy),   2*(yz-wx),   1-2*(xx+yy), 0,
        0,           0,           0,           1
    ]
    return matrix

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

    # Display orientation values as text (standard RPY order)
    osd_line = f"roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}"
    drawText((-1,-2, 2), osd_line)

    # Apply quaternion rotation using rotation matrix (no gimbal lock!)
    rot_matrix = quat_to_matrix(quat_w, quat_x, quat_y, quat_z)
    glMultMatrixf(rot_matrix)

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
# PERFORMANCE: Most prints commented out for smooth 60Hz animation

def on_config_ack():
    """Called when configuration acknowledgment is received (0x12)"""
    print("Configuration set successfully.")

def on_wakeup_ack():
    """Called when sensor wake-up acknowledgment is received (0x03)"""
    print("Sensor woken up.")

def on_reporting_enabled_ack():
    """Called when proactive reporting enabled acknowledgment is received (0x19)"""
    print("Proactive reporting enabled. Starting visualization...")

def on_quaternion(w, x, y, z):
    """Called when quaternion data is received - updates visualization (no gimbal lock!)"""
    global quat_w, quat_x, quat_y, quat_z
    
    # Apply axis remapping (same as Kivy app: w, -x, z, y)
    quat_w = w
    quat_x = y
    quat_y = z
    quat_z = x
    
    # Redraw the 3D visualization
    draw()
    pygame.display.flip()

def on_euler_angles(roll_val, pitch_val, yaw_val):
    """Called when Euler angle data is received - store for display"""
    global roll, pitch, yaw
    
    # Store Euler angles for on-screen display
    roll = roll_val
    pitch = pitch_val
    yaw = yaw_val

def on_unknown_command(command_id):
    """Called when unknown command ID is received"""
    print(f"Error! Command ID not defined: 0x{command_id:02X}.")

# Create callbacks dictionary for the IMU parser
imu_callbacks = {
    'config_ack': on_config_ack,
    'wakeup_ack': on_wakeup_ack,
    'reporting_enabled_ack' : on_reporting_enabled_ack,
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

        resize(1024,768)
        init()

        # Loop to receive data and process it
        print("\nVisualization running - orientation values shown on screen")
        print("Press ESC to quit\n")
        
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