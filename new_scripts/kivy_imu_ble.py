import sys
import asyncio
import threading
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.graphics import Color, Mesh, Rectangle
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


class IMU3DVisualizer(Widget):
    """Widget that displays a 3D representation of the IMU orientation"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.quat_w = 1.0
        self.quat_x = 0.0
        self.quat_y = 0.0
        self.quat_z = 0.0
        self.bind(pos=self.update_canvas, size=self.update_canvas)
        
        # Define faces with consistent winding order
        self.faces = [
            # Top (Y+) - green
            (( 1.0,  0.2,  1.0), (-1.0,  0.2,  1.0), (-1.0,  0.2, -1.0), ( 1.0,  0.2, -1.0)),
            # Bottom (Y-) - orange  
            (( 1.0, -0.2, -1.0), (-1.0, -0.2, -1.0), (-1.0, -0.2,  1.0), ( 1.0, -0.2,  1.0)),
            # Front (Z+) - red
            (( 1.0,  0.2,  1.0), ( 1.0, -0.2,  1.0), (-1.0, -0.2,  1.0), (-1.0,  0.2,  1.0)),
            # Back (Z-) - yellow
            ((-1.0,  0.2, -1.0), (-1.0, -0.2, -1.0), ( 1.0, -0.2, -1.0), ( 1.0,  0.2, -1.0)),
            # Left (X-) - blue
            ((-1.0,  0.2,  1.0), (-1.0, -0.2,  1.0), (-1.0, -0.2, -1.0), (-1.0,  0.2, -1.0)),
            # Right (X+) - magenta
            (( 1.0,  0.2, -1.0), ( 1.0, -0.2, -1.0), ( 1.0, -0.2,  1.0), ( 1.0,  0.2,  1.0)),
        ]
        
        self.colors = [
            (0.0, 1.0, 0.0),  # Top - green
            (1.0, 0.5, 0.0),  # Bottom - orange
            (1.0, 0.0, 0.0),  # Front - red
            (1.0, 1.0, 0.0),  # Back - yellow
            (0.0, 0.0, 1.0),  # Left - blue
            (1.0, 0.0, 1.0),  # Right - magenta
        ]
        
        self.update_canvas()
    
    def set_quaternion(self, w, x, y, z):
        """Update orientation using quaternion"""
        # Apply axis remapping (matching PyGame version: w, y, z, x)
        self.quat_w = w
        self.quat_x = y
        self.quat_y = z
        self.quat_z = x
        self.update_canvas()
    
    def _rotate_point_quat(self, x, y, z):
        """Rotate point using quaternion"""
        w, qx, qy, qz = self.quat_w, self.quat_x, self.quat_y, self.quat_z
        
        # Normalize quaternion
        norm = (w*w + qx*qx + qy*qy + qz*qz) ** 0.5
        if norm > 0:
            w, qx, qy, qz = w/norm, qx/norm, qy/norm, qz/norm
        
        # Quaternion rotation formula: v' = v + 2*q.xyz × (q.xyz × v + q.w*v)
        cx1 = qy * z - qz * y
        cy1 = qz * x - qx * z
        cz1 = qx * y - qy * x
        
        cx1 += w * x
        cy1 += w * y
        cz1 += w * z
        
        cx2 = qy * cz1 - qz * cy1
        cy2 = qz * cx1 - qx * cz1
        cz2 = qx * cy1 - qy * cx1
        
        return x + 2.0 * cx2, y + 2.0 * cy2, z + 2.0 * cz2
    
    def _compute_normal(self, v0, v1, v2):
        """Compute face normal from three vertices using cross product"""
        e1 = (v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2])
        e2 = (v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2])
        
        nx = e1[1] * e2[2] - e1[2] * e2[1]
        ny = e1[2] * e2[0] - e1[0] * e2[2]
        nz = e1[0] * e2[1] - e1[1] * e2[0]
        
        return nx, ny, nz
    
    def _project_3d_to_2d(self, x, y, z, scale, cx, cy):
        """Simple orthographic projection"""
        return cx + x * scale, cy + y * scale, z
    
    def update_canvas(self, *args):
        """Redraw the 3D visualization"""
        self.canvas.clear()
        
        cx = self.center_x
        cy = self.center_y
        scale = min(self.width, self.height) / 2.5
        
        # Process each face
        visible_faces = []
        
        for face_idx, face in enumerate(self.faces):
            # Rotate all vertices
            rotated = [self._rotate_point_quat(v[0], v[1], v[2]) for v in face]
            
            # Compute normal of rotated face
            nx, ny, nz = self._compute_normal(rotated[0], rotated[1], rotated[2])
            
            # Backface culling: only draw if face is visible from camera
            if nz >= 0:
                continue
            
            # Project to 2D
            projected = [self._project_3d_to_2d(r[0], r[1], r[2], scale, cx, cy) for r in rotated]
            
            # Calculate depth for sorting (average Z)
            avg_z = sum(r[2] for r in rotated) / 4
            
            visible_faces.append((avg_z, face_idx, projected))
        
        # Sort by depth (furthest first for painter's algorithm)
        visible_faces.sort(key=lambda f: f[0])
        
        # Draw faces
        with self.canvas:
            # Black background
            Color(0, 0, 0, 1)
            Rectangle(pos=self.pos, size=self.size)
            
            # Draw each visible face
            for avg_z, face_idx, verts in visible_faces:
                color = self.colors[face_idx]
                Color(color[0], color[1], color[2], 1.0)
                
                # Use Mesh with triangle fan mode for quad
                vertices = []
                for v in verts:
                    vertices.extend([v[0], v[1], 0, 0])  # x, y, u, v
                
                indices = [0, 1, 2, 0, 2, 3]  # Two triangles for quad
                
                Mesh(vertices=vertices, indices=indices, mode='triangles')


class BLEWidget(BoxLayout):
    def __init__(self, device_addr, **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'vertical'
        
        self.device_addr = device_addr
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Status label
        self.status_label = Label(
            text=f"Press 'Connect' to start ({self.device_addr})",
            size_hint=(1, 0.1),
            color=(1, 1, 1, 1)
        )
        self.add_widget(self.status_label)
        
        # Orientation display (standard RPY order)
        self.orientation_label = Label(
            text="roll: 0.00, pitch: 0.00, yaw: 0.00",
            size_hint=(1, 0.1),
            color=(0.5, 1, 0.5, 1)
        )
        self.add_widget(self.orientation_label)
        
        # 3D visualization
        self.visualizer = IMU3DVisualizer(size_hint=(1, 0.7))
        self.add_widget(self.visualizer)
        
        # Connect button
        self.connect_button = Button(
            text="Connect",
            size_hint=(1, 0.1),
            background_color=(0.2, 0.6, 0.2, 1)
        )
        self.connect_button.bind(on_press=self.on_connect)
        self.add_widget(self.connect_button)
        
        # BLE state
        self.client = None
        
        # Setup IMU callbacks
        self.imu_callbacks = {
            'config_ack': self.on_config_ack,
            'wakeup_ack': self.on_wakeup_ack,
            'reporting_enabled_ack': self.on_reporting_enabled_ack,
            'ble_stay_connected_ack': self.on_ble_stay_connected_ack,
            'ble_highspeed_ack': self.on_ble_highspeed_ack,
            'quaternion': self.on_quaternion,
            'euler_angles': self.on_euler_angles,
            'unknown_command': self.on_unknown_command,
        }

    def update_status(self, text):
        Clock.schedule_once(lambda dt: setattr(self.status_label, 'text', text))

    def update_orientation_display(self):
        text = f"roll: {self.roll:.2f}, pitch: {self.pitch:.2f}, yaw: {self.yaw:.2f}"
        Clock.schedule_once(lambda dt: setattr(self.orientation_label, 'text', text))

    # ========== IMU Data Callbacks ==========
    
    def on_config_ack(self):
        self.update_status("Configuration set successfully")
        print("Configuration set successfully.")

    def on_wakeup_ack(self):
        self.update_status("Sensor woken up")
        print("Sensor woken up.")

    def on_reporting_enabled_ack(self):
        self.update_status("Streaming IMU data...")
        print("Proactive reporting enabled.")

    def on_ble_stay_connected_ack(self):
        print("BLE stay connected enabled.")

    def on_ble_highspeed_ack(self):
        print("BLE high-speed communication enabled.")

    def on_quaternion(self, w, x, y, z):
        Clock.schedule_once(lambda dt, w=w, x=x, y=y, z=z: 
                            self.visualizer.set_quaternion(w, x, y, z), 0)

    def on_euler_angles(self, roll_val, pitch_val, yaw_val):
        self.roll = roll_val
        self.pitch = pitch_val
        self.yaw = yaw_val
        Clock.schedule_once(lambda dt: self.update_orientation_display(), 0)

    def on_unknown_command(self, command_id):
        print(f"Unknown command: 0x{command_id:02X}")

    def notification_handler(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        Cmd_RxUnpack(data, len(data), callbacks=self.imu_callbacks)

    def run_ble_in_thread(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        max_attempts = 5
        
        for attempt in range(max_attempts):
            try:
                print(f"Connection attempt {attempt + 1}/{max_attempts}")
                self.update_status(f"Connection attempt {attempt + 1}/{max_attempts}...")
                loop.run_until_complete(self.ble_task())
                break
            except Exception as e:
                print(f"BLE Error: {e}")
                self.update_status(f"Error: {e}")
                if attempt < max_attempts - 1:
                    print("Retrying in 3 seconds...")
                    import time
                    time.sleep(3)
        else:
            print(f"Failed to connect after {max_attempts} attempts")
            self.update_status(f"Failed after {max_attempts} attempts")
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'disabled', False), 0)
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'text', 'Connect'), 0)
        
        loop.close()

    async def ble_task(self):
        self.update_status("Scanning for IMU device...")
        print("Starting scan...")
        
        device = await BleakScanner.find_device_by_address(
            self.device_addr, cb=dict(use_bdaddr=False)
        )

        if device is None:
            self.update_status(f"Device {self.device_addr} not found!")
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'disabled', False), 0)
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'text', 'Connect'), 0)
            print(f"Could not find device with address {self.device_addr}")
            return

        self.update_status("Connecting...")
        print("Connecting to device...")
        
        disconnected_event = asyncio.Event()

        def disconnected_callback(client):
            print("Disconnected callback called!")
            self.update_status("Disconnected")
            disconnected_event.set()
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'disabled', False), 0)
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'text', 'Connect'), 0)

        async with BleakClient(device, disconnected_callback=disconnected_callback) as client:
            self.client = client
            self.update_status("Connected! Initializing...")
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'text', 'Disconnect'), 0)
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'disabled', False), 0)
            print("Connected")
            
            await client.start_notify(par_notification_characteristic, self.notification_handler)
            
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

            await client.write_gatt_char(par_write_characteristic, bytes([0x19]))

            self.update_status("Streaming IMU data...")
            print("Receiving data...")

            await disconnected_event.wait()

        self.update_status(f"Disconnected ({self.device_addr})")
        print("Disconnected")

    def on_connect(self, instance):
        if self.connect_button.text == 'Disconnect':
            # TODO: Implement disconnect
            pass
        else:
            self.connect_button.disabled = True
            self.connect_button.text = "Connecting..."
            self.update_status("Launching BLE connection...")
            
            thread = threading.Thread(target=self.run_ble_in_thread, daemon=True)
            thread.start()


class BLEIMUApp(App):
    def __init__(self, device_addr, **kwargs):
        super().__init__(**kwargs)
        self.device_addr = device_addr
    
    def build(self):
        self.title = "IMU Visualization (BLE) - Kivy"
        return BLEWidget(self.device_addr)


if __name__ == "__main__":
    device_addr = par_device_addr
    
    if len(sys.argv) > 1:
        device_addr = sys.argv[1]
    
    print(f"IMU Visualization (BLE) - Target: {device_addr}")
    print("Press 'Connect' button to start streaming\n")
    
    BLEIMUApp(device_addr).run()