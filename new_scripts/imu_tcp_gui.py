import socket
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *

# TCP connection settings
TCP_HOST = "192.168.0.102"  # ESP32 IP address - adjust as needed
TCP_PORT = 25

# Global orientation variables
quat_w, quat_x, quat_y, quat_z = 1.0, 0.0, 0.0, 0.0
roll, pitch, yaw = 0.0, 0.0, 0.0

# Initialize pygame and OpenGL
pygame.init()
screen = pygame.display.set_mode((1024, 768), OPENGL | DOUBLEBUF)
pygame.display.set_caption("IMU Visualization (TCP) - Press Esc to quit")


def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init_gl():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def quat_to_matrix(w, x, y, z):
    """Convert quaternion to 4x4 OpenGL rotation matrix"""
    norm = (w*w + x*x + y*y + z*z) ** 0.5
    if norm > 0:
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    
    return [
        1-2*(yy+zz), 2*(xy+wz),   2*(xz-wy),   0,
        2*(xy-wz),   1-2*(xx+zz), 2*(yz+wx),   0,
        2*(xz+wy),   2*(yz-wx),   1-2*(xx+yy), 0,
        0,           0,           0,           1
    ]


def draw_text(position, text):
    font = pygame.font.SysFont("Courier", 18, True)
    surface = font.render(text, True, (255, 255, 255, 255), (0, 0, 0, 255))
    data = pygame.image.tostring(surface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(surface.get_width(), surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, data)


def draw():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    # Display Euler angles (standard RPY order)
    osd = f"roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}"
    draw_text((-1, -2, 2), osd)

    # Apply quaternion rotation
    rot_matrix = quat_to_matrix(quat_w, quat_x, quat_y, quat_z)
    glMultMatrixf(rot_matrix)

    # Draw colored box representing IMU
    glBegin(GL_QUADS)
    # Top (green)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)
    # Bottom (orange)
    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)
    # Front (red)
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    # Back (yellow)
    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)
    # Left (blue)
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    # Right (magenta)
    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def parse_imu_line(line):
    """
    Parse CSV line from ESP32 TCP server.
    Format: ts,ang[3],awg[3],gyro[3],mag[3],quat[4],euler[3],temp,press,height
    Indices: 0, 1-3,   4-6,   7-9,    10-12, 13-16,  17-19,   20,  21,   22
    """
    global quat_w, quat_x, quat_y, quat_z, roll, pitch, yaw
    
    parts = line.strip().split(',')
    if len(parts) < 20:
        return False
    
    try:
        # Quaternion at indices 13-16 (w, x, y, z)
        w = float(parts[13])
        x = float(parts[14])
        y = float(parts[15])
        z = float(parts[16])
        
        # Apply axis remapping (matching original: w, -x, z, y)
        quat_w = w
        quat_x = y
        quat_y = z
        quat_z = x
        
        # Euler angles at indices 17-19 (roll, pitch, yaw)
        roll = float(parts[17])
        pitch = float(parts[18])
        yaw = float(parts[19])
        
        return True
    except:
        return False


def main():
    global TCP_HOST
    
    # Allow command line override of host
    import sys
    if len(sys.argv) > 1:
        TCP_HOST = sys.argv[1]
    
    print(f"Connecting to {TCP_HOST}:{TCP_PORT}...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    
    try:
        sock.connect((TCP_HOST, TCP_PORT))
        print("Connected!")
    except socket.error as e:
        print(f"Connection failed: {e}")
        return
    
    resize(1024, 768)
    init_gl()
    
    print("\nVisualization running - Press ESC to quit\n")
    
    buf = ""
    running = True
    sock.settimeout(0.5)
    
    while running:
        # Handle pygame events
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                running = False
        
        # Read from socket
        try:
            data = sock.recv(1024).decode(errors='ignore')
        except socket.timeout:
            continue
        except socket.error as e:
            print(f"Socket error: {e}")
            break
        
        if not data:
            print("Connection closed by server")
            break
        
        buf += data
        
        # Process complete lines using partition (like UWB script)
        while "\n" in buf:
            line, _, buf = buf.partition("\n")
            if parse_imu_line(line):
                draw()
                pygame.display.flip()
    
    sock.close()
    pygame.quit()
    print("Done.")


if __name__ == "__main__":
    main()