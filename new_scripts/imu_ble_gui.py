import sys
import asyncio
import threading
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

# Import the reusable IMU parser library
from imu_parser import Cmd_RxUnpack

# BLE parameters (adjust to your device)
# Characteristic UUID of the device
# par_notification_characteristic="0000ae02-0000-1000-8000-00805f9b34fb"
par_notification_characteristic = 0x0007
# Characteristic UUID of the device (with write attribute Write)
# par_write_characteristic="0000ae01-0000-1000-8000-00805f9b34fb"
par_write_characteristic = 0x0005

par_device_addr = "AC:25:DD:6E:69:4D"  # Change to your device MAC

# Global orientation variables (quaternion for rotation)
quat_w = 1.0
quat_x = 0.0
quat_y = 0.0
quat_z = 0.0

# Euler angles for display only
roll = 0.0
pitch = 0.0
yaw = 0.0

# Connection state
connected = False
running = True


def resize(width, height):
    """Setup OpenGL viewport and projection"""
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init_gl():
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


def draw_text(position, text):
    """Draw text on the OpenGL screen"""
    font = pygame.font.SysFont("Courier", 18, True)
    surface = font.render(text, True, (255, 255, 255, 255), (0, 0, 0, 255))
    data = pygame.image.tostring(surface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(surface.get_width(), surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, data)


def draw():
    """Render the 3D IMU visualization"""
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    # Display orientation values as text (standard RPY order)
    osd_line = f"roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}"
    draw_text((-1, -2, 2), osd_line)

    # Apply quaternion rotation using rotation matrix (no gimbal lock!)
    rot_matrix = quat_to_matrix(quat_w, quat_x, quat_y, quat_z)
    glMultMatrixf(rot_matrix)

    # Draw a colored box representing the IMU board
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
    print("Proactive reporting enabled. Starting visualization...")


def on_ble_stay_connected_ack():
    """Called when BLE stay connected acknowledgment is received (0x29)"""
    print("BLE stay connected enabled.")


def on_ble_highspeed_ack():
    """Called when BLE high-speed communication acknowledgment is received (0x46)"""
    print("BLE high-speed communication enabled.")


def on_quaternion(w, x, y, z):
    """Called when quaternion data is received - updates visualization (no gimbal lock!)"""
    global quat_w, quat_x, quat_y, quat_z
    
    # Apply axis remapping (same as other apps: w, y, z, x)
    quat_w = w
    quat_x = y
    quat_y = z
    quat_z = x


def on_euler_angles(roll_val, pitch_val, yaw_val):
    """Called when Euler angle data is received - store for display"""
    global roll, pitch, yaw
    
    # Store Euler angles for on-screen display
    roll = roll_val
    pitch = pitch_val
    yaw = yaw_val


def on_unknown_command(command_id):
    """Called when unknown command ID is received"""
    print(f"Error! Command ID not defined: 0x{command_id:02X}")


# Create callbacks dictionary for the IMU parser
imu_callbacks = {
    'config_ack': on_config_ack,
    'wakeup_ack': on_wakeup_ack,
    'reporting_enabled_ack': on_reporting_enabled_ack,
    'ble_stay_connected_ack': on_ble_stay_connected_ack,
    'ble_highspeed_ack': on_ble_highspeed_ack,
    'quaternion': on_quaternion,
    'euler_angles': on_euler_angles,
    'unknown_command': on_unknown_command,
}


def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    """Called when BLE notification data is received"""
    # BLE delivers the payload directly (no serial packet framing needed)
    Cmd_RxUnpack(data, len(data), callbacks=imu_callbacks)


async def ble_task(device_addr):
    """Main BLE connection and communication task"""
    global connected, running
    
    print(f"Scanning for device {device_addr}...")
    
    device = await BleakScanner.find_device_by_address(
        device_addr, cb=dict(use_bdaddr=False)
    )

    if device is None:
        print(f"Could not find device with address {device_addr}")
        return

    print("Connecting to device...")
    
    disconnected_event = asyncio.Event()

    def disconnected_callback(client):
        print("Disconnected callback called!")
        disconnected_event.set()

    async with BleakClient(device, disconnected_callback=disconnected_callback) as client:
        connected = True
        print("Connected!")
        
        # Start notifications
        await client.start_notify(par_notification_characteristic, notification_handler)
        
        # Send initialization commands
        await client.write_gatt_char(par_write_characteristic, bytes([0x29]))
        await client.write_gatt_char(par_write_characteristic, bytes([0x46]))

        # Configure IMU parameters
        isCompassOn = 0  # Whether to use magnetic field fusion
        barometerFilter = 2
        Cmd_ReportTag = 0x7F  # Feature subscription tag
        params = bytearray([0x00 for i in range(0, 11)])
        params[0] = 0x12
        params[1] = 5       # Stationary state acceleration threshold
        params[2] = 255     # Static zero return speed (unit cm/s) 0: No return to zero 255: Return to zero immediately
        params[3] = 0       # Dynamic zero return speed (unit cm/s) 0: No return to zero
        params[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1)
        params[5] = 60      # The transmission frame rate of data actively reported [value 0-250HZ], 0 means 0.5HZ
        params[6] = 1       # Gyroscope filter coefficient [value 0-2], the larger the value, the more stable it is but the worse the real-time performance.
        params[7] = 3       # Accelerometer filter coefficient [value 0-4], the larger the value, the more stable it is but the worse the real-time performance.
        params[8] = 5       # Magnetometer filter coefficient [value 0-9], the larger the value, the more stable it is but the worse the real-time performance.
        params[9] = Cmd_ReportTag & 0xff
        params[10] = (Cmd_ReportTag >> 8) & 0xff
        await client.write_gatt_char(par_write_characteristic, params)

        # Enable proactive reporting
        await client.write_gatt_char(par_write_characteristic, bytes([0x19]))

        print("Receiving data...")

        # Wait until disconnected or main loop exits
        while running and not disconnected_event.is_set():
            await asyncio.sleep(0.1)

    connected = False
    print("BLE disconnected.")


def run_ble_thread(device_addr):
    """Run BLE connection in a separate thread with its own event loop"""
    global running
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    max_attempts = 5
    
    for attempt in range(max_attempts):
        if not running:
            break
        try:
            print(f"Connection attempt {attempt + 1}/{max_attempts}")
            loop.run_until_complete(ble_task(device_addr))
            break
        except Exception as e:
            print(f"BLE Error: {e}")
            if attempt < max_attempts - 1 and running:
                print("Retrying in 3 seconds...")
                import time
                time.sleep(3)
    else:
        print(f"Failed to connect after {max_attempts} attempts")
    
    loop.close()


def main():
    global running
    
    # Allow command line override of device address
    device_addr = par_device_addr
    if len(sys.argv) > 1:
        device_addr = sys.argv[1]
    
    print("------------BLE IMU Visualization--------------")
    print(f"Target device: {device_addr}")
    
    # Initialize pygame and OpenGL
    pygame.init()
    screen = pygame.display.set_mode((1024, 768), OPENGL | DOUBLEBUF)
    pygame.display.set_caption("IMU Visualization (BLE) - Press Esc to quit")
    
    resize(1024, 768)
    init_gl()
    
    # Start BLE in separate thread
    ble_thread = threading.Thread(target=run_ble_thread, args=(device_addr,), daemon=True)
    ble_thread.start()
    
    print("\nVisualization running - Press ESC to quit\n")
    
    # Main loop
    clock = pygame.time.Clock()
    
    while running:
        # Handle pygame events
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                running = False
        
        # Render
        draw()
        pygame.display.flip()
        
        # Limit frame rate
        clock.tick(60)
    
    pygame.quit()
    print("Done.")


if __name__ == "__main__":
    main()