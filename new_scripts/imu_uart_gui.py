import serial

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *

# Import the reusable IMU parser library
from imu_parser import Cmd_GetPkt, Cmd_PackAndTx, handle_response


# ================== Serial configuration ==================
ser_port = "COM18"
ser_baudrate = 115200
ser_timeout = 2


# ================== Global orientation state ==================
quat_w = 1.0
quat_x = 0.0
quat_y = 0.0
quat_z = 0.0

roll = 0.0
pitch = 0.0
yaw = 0.0


# ================== Pygame / OpenGL setup ==================
video_flags = OPENGL | DOUBLEBUF
pygame.init()
screen = pygame.display.set_mode((1024, 768), video_flags)
pygame.display.set_caption("IMU Visualization (NED) – Press ESC to quit")


CUBE_HALF_X = 1.0
CUBE_HALF_Y = 0.2
CUBE_HALF_Z = 1.0
AXIS_EXTENSION = 0.5   # how far OUTSIDE the cube each axis should extend

def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def quat_to_matrix(w, x, y, z):
    norm = (w*w + x*x + y*y + z*z)**0.5
    if norm > 0:
        w, x, y, z = w/norm, x/norm, y/norm, z/norm

    xx, yy, zz = x*x, y*y, z*z
    xy = x*y; xz = x*z; yz = y*z
    wx = w*x; wy = w*y; wz = w*z

    return [
        1 - 2*(yy + zz),   2*(xy + wz),     2*(xz - wy),     0,
        2*(xy - wz),       1 - 2*(xx + zz), 2*(yz + wx),     0,
        2*(xz + wy),       2*(yz - wx),     1 - 2*(xx + yy), 0,
        0,                 0,               0,               1
    ]


def draw_text(position, textString):
    font = pygame.font.SysFont("Courier", 18, True)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA,
                 GL_UNSIGNED_BYTE, textData)

# ============================================================
#  NED AXES (BODY FRAME, ROTATES WITH IMU)
# ============================================================
def draw_ned_body_axes():
    glLineWidth(4)

    # X_ned forward → GL -Z
    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, -CUBE_HALF_Z)
    glVertex3f(0, 0, -CUBE_HALF_Z - AXIS_EXTENSION)
    glEnd()

    # Y_ned right → GL -X
    glColor3f(0.0, 1.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(-CUBE_HALF_X, 0, 0)
    glVertex3f(-CUBE_HALF_X - AXIS_EXTENSION, 0, 0)
    glEnd()

    # Z_ned down → GL +Y
    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3f(0, CUBE_HALF_Y, 0)
    glVertex3f(0, CUBE_HALF_Y + AXIS_EXTENSION, 0)
    glEnd()


# ============================================================
#  WORLD REFERENCE NED AXES (FIXED IN SCREEN)
# ============================================================
def draw_world_ned_axes(length=0.8):
    glLineWidth(3)

    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0); glVertex3f(0, 0, -length)
    glEnd()

    glColor3f(0.0, 1.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0); glVertex3f(length, 0, 0)
    glEnd()

    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0); glVertex3f(0, -length, 0)
    glEnd()

def draw_gl_axes(length=0.8):
    glLineWidth(3)

    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0); glVertex3f(length, 0, 0)
    glEnd()

    glColor3f(0.0, 1.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0); glVertex3f(0, length, 0)
    glEnd()

    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0); glVertex3f(0, 0, length)
    glEnd()


# ============================================================
#  CUBE MODEL
# ============================================================
def draw_board():
    glBegin(GL_QUADS)

    # Left (blue)
    glColor3f(0.0,0.0,1.0)
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f( 1.0, 0.2, 1.0)

    # Bottom (orange)
    glColor3f(1.0,0.5,0.0)
    glVertex3f( 1.0,-0.2, 1.0)
    glVertex3f(-1.0,-0.2, 1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f( 1.0,-0.2,-1.0)

    # Back (yellow)
    glColor3f(1.0,1.0,0.0)
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0,-0.2, 1.0)
    glVertex3f( 1.0,-0.2, 1.0)

    # Front (red)
    glColor3f(1.0,0.0,0.0)
    glVertex3f( 1.0,-0.2,-1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)
    glVertex3f( 1.0, 0.2,-1.0)

    # Top (green)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2,-1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f(-1.0,-0.2, 1.0)

    # Right (magenta)
    glColor3f(1.0,0.0,1.0)
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f( 1.0,-0.2, 1.0)
    glVertex3f( 1.0,-0.2,-1.0)

    glEnd()


# ============================================================
# DRAW SCENE
# ============================================================
def draw():
    global roll, pitch, yaw

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0.0, 0.0, -7.0)

    draw_text((-1, -2, 2), f"roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}")

    glPushMatrix()
    glTranslatef(-6.5, -5.0, -6.0)
    draw_gl_axes()
    glPopMatrix()

    glPushMatrix()
    glTranslatef(-6.5, -3.0, -6.0)
    draw_world_ned_axes()
    glPopMatrix()

    # Apply rotation from IMU
    rot_matrix = quat_to_matrix(quat_w, quat_x, quat_y, quat_z)
    glPushMatrix()
    glMultMatrixf(rot_matrix)

    # Draw rotating body-frame axes
    draw_ned_body_axes()

    # Draw the cube
    draw_board()

    glPopMatrix()


# ============================================================
# IMU CALLBACKS
# ============================================================
def on_quaternion(w, x, y, z):
    global quat_w, quat_x, quat_y, quat_z
    # Your working mapping — DO NOT CHANGE
    quat_w = w
    quat_x = -y
    quat_y =  z
    quat_z = -x

    draw()
    pygame.display.flip()


def on_euler_angles(roll_val, pitch_val, yaw_val):
    global roll, pitch, yaw
    roll = roll_val
    pitch = pitch_val
    yaw = yaw_val


def on_config_ack(): print("Configuration set OK.")
def on_wakeup_ack(): print("Sensor woke up.")
def on_reporting_enabled_ack(): print("Proactive reporting enabled.")
def on_unknown_command(cmd): print(f"Unknown command 0x{cmd:02X}")


imu_callbacks = {
    "config_ack": on_config_ack,
    "wakeup_ack": on_wakeup_ack,
    "reporting_enabled_ack": on_reporting_enabled_ack,
    "quaternion": on_quaternion,
    "euler_angles": on_euler_angles,
    "unknown_command": on_unknown_command,
}

# ============================================================
# MAIN LOOP
# ============================================================
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
