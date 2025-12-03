import asyncio
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.graphics import Color, Quad
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
import math
import threading

# Import the reusable IMU parser library
from imu_parser import Cmd_RxUnpack

# BLE parameters (adjust to your device)
par_notification_characteristic = 0x0007
par_write_characteristic = 0x0005
par_device_addr = "AC:25:DD:6E:69:4D"  # Change to your device MAC


class IMU3DVisualizer(Widget):
    """Widget that displays a 3D representation of the IMU orientation"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # Store quaternion
        self.quat_w = 1.0
        self.quat_x = 0.0
        self.quat_y = 0.0
        self.quat_z = 0.0
        self.bind(pos=self.update_canvas, size=self.update_canvas)
        self.update_canvas()
    
    def set_quaternion(self, w, x, y, z):
        """Update orientation using quaternion (avoids gimbal lock!)"""
        # Correct IMU to OpenGL coordinate system mapping
        # This makes yaw, pitch, and roll all rotate correctly
        self.quat_w, self.quat_x, self.quat_y, self.quat_z = w, -x, z, y
        
        self.update_canvas()
    
    def _rotate_point_quat(self, x, y, z):
        """Rotate point using quaternion (no gimbal lock!)"""
        # Convert quaternion to rotation
        w, qx, qy, qz = self.quat_w, self.quat_x, self.quat_y, self.quat_z
        
        # Quaternion rotation formula
        # v' = v + 2*q.xyz × (q.xyz × v + q.w*v)
        
        # First cross product: q.xyz × v
        cx1 = qy * z - qz * y
        cy1 = qz * x - qx * z
        cz1 = qx * y - qy * x
        
        # Add q.w * v
        cx1 += w * x
        cy1 += w * y
        cz1 += w * z
        
        # Second cross product: q.xyz × result
        cx2 = qy * cz1 - qz * cy1
        cy2 = qz * cx1 - qx * cz1
        cz2 = qx * cy1 - qy * cx1
        
        # Final result
        rx = x + 2.0 * cx2
        ry = y + 2.0 * cy2
        rz = z + 2.0 * cz2
        
        return rx, ry, rz
    
    def _project_3d_to_2d(self, x, y, z, scale, cx, cy):
        """Simple orthographic projection - no perspective"""
        screen_x = cx + x * scale
        screen_y = cy + y * scale
        return screen_x, screen_y, z
    
    def update_canvas(self, *args):
        """Redraw the 3D visualization"""
        from kivy.graphics import Quad
        
        self.canvas.clear()
        
        # Center and scale - make it MUCH larger
        cx = self.center_x
        cy = self.center_y
        scale = min(self.width, self.height) / 2.5  # Much larger!
        
        # Define box vertices (matching OpenGL version proportions)
        # Width=2.0, Height=0.4, Depth=2.0 (like the OpenGL version)
        vertices = [
            # Top face
            ( 1.0,  0.2, -1.0), (-1.0,  0.2, -1.0), (-1.0,  0.2,  1.0), ( 1.0,  0.2,  1.0),
            # Bottom face  
            ( 1.0, -0.2,  1.0), (-1.0, -0.2,  1.0), (-1.0, -0.2, -1.0), ( 1.0, -0.2, -1.0),
            # Front face
            ( 1.0,  0.2,  1.0), (-1.0,  0.2,  1.0), (-1.0, -0.2,  1.0), ( 1.0, -0.2,  1.0),
            # Back face
            ( 1.0, -0.2, -1.0), (-1.0, -0.2, -1.0), (-1.0,  0.2, -1.0), ( 1.0,  0.2, -1.0),
            # Left face
            (-1.0,  0.2,  1.0), (-1.0,  0.2, -1.0), (-1.0, -0.2, -1.0), (-1.0, -0.2,  1.0),
            # Right face
            ( 1.0,  0.2, -1.0), ( 1.0,  0.2,  1.0), ( 1.0, -0.2,  1.0), ( 1.0, -0.2, -1.0),
        ]
        
        # Face colors (matching OpenGL version)
        colors = [
            (0.0, 1.0, 0.0),  # Top - green
            (1.0, 0.5, 0.0),  # Bottom - orange
            (1.0, 0.0, 0.0),  # Front - red
            (1.0, 1.0, 0.0),  # Back - yellow
            (0.0, 0.0, 1.0),  # Left - blue
            (1.0, 0.0, 1.0),  # Right - magenta
        ]
        
        # Rotate and project all vertices
        projected_faces = []
        for face_idx in range(6):
            face_vertices = []
            for vert_idx in range(4):
                v = vertices[face_idx * 4 + vert_idx]
                # Use quaternion rotation (no gimbal lock!)
                rx, ry, rz = self._rotate_point_quat(v[0], v[1], v[2])
                # Project to 2D
                px, py, pz = self._project_3d_to_2d(rx, ry, rz, scale, cx, cy)
                face_vertices.append((px, py, pz))
            
            # Calculate average Z for depth sorting
            avg_z = sum(v[2] for v in face_vertices) / 4
            projected_faces.append((avg_z, face_idx, face_vertices))
        
        # Sort faces by Z (back to front)
        projected_faces.sort(key=lambda f: f[0])
        
        # Draw faces back to front
        with self.canvas:
            for avg_z, face_idx, face_verts in projected_faces:
                color = colors[face_idx]
                Color(color[0], color[1], color[2], 1.0)
                
                # Draw as a quad
                Quad(points=[
                    face_verts[0][0], face_verts[0][1],
                    face_verts[1][0], face_verts[1][1],
                    face_verts[2][0], face_verts[2][1],
                    face_verts[3][0], face_verts[3][1],
                ])


class BLEWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'vertical'
        
        # Global orientation variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Status label at top
        self.status_label = Label(
            text="Press 'Connect to IMU' to start",
            size_hint=(1, 0.1),
            color=(1, 1, 1, 1)
        )
        self.add_widget(self.status_label)
        
        # Orientation values display
        self.orientation_label = Label(
            text="Roll: 0.00° | Pitch: 0.00° | Yaw: 0.00°",
            size_hint=(1, 0.1),
            color=(0.5, 1, 0.5, 1)
        )
        self.add_widget(self.orientation_label)
        
        # 3D visualization widget
        self.visualizer = IMU3DVisualizer(size_hint=(1, 0.7))
        self.add_widget(self.visualizer)
        
        # Control button at bottom
        self.start_button = Button(
            text="Connect to IMU",
            size_hint=(1, 0.1),
            background_color=(0.2, 0.6, 0.2, 1)
        )
        self.start_button.bind(on_press=self.on_start_ble)
        self.add_widget(self.start_button)
        
        # BLE state
        self.client = None
        
        # Setup IMU callbacks
        self.imu_callbacks = {
            'config_ack': self.on_config_ack,
            'wakeup_ack': self.on_wakeup_ack,
            'reporting_enabled_ack': self.on_reporting_enabled_ack,
            'ble_stay_connected_ack': self.on_ble_stay_connected_ack,
            'ble_highspeed_ack': self.on_ble_highspeed_ack,
            'packet_header': self.on_packet_header,
            'accel_no_gravity': self.on_accel_no_gravity,
            'accel_with_gravity': self.on_accel_with_gravity,
            'gyroscope': self.on_gyroscope,
            'magnetometer': self.on_magnetometer,
            'environmental': self.on_environmental,
            'quaternion': self.on_quaternion,
            'euler_angles': self.on_euler_angles,
            'unknown_command': self.on_unknown_command,
        }

    def update_status(self, text):
        """Update the status label on the main thread"""
        Clock.schedule_once(lambda dt: setattr(self.status_label, 'text', text))

    def update_orientation_display(self):
        """Update the orientation values display"""
        text = f"Roll: {self.roll:.2f}° | Pitch: {self.pitch:.2f}° | Yaw: {self.yaw:.2f}°"
        Clock.schedule_once(lambda dt: setattr(self.orientation_label, 'text', text))

    # ========== IMU Data Callbacks (called by imu_parser) ==========
    
    def on_config_ack(self):
        """Called when configuration acknowledgment is received (0x12)"""
        self.update_status("Configuration set successfully")
        print("Configuration set successfully.")

    def on_wakeup_ack(self):
        """Called when sensor wake-up acknowledgment is received (0x03)"""
        self.update_status("Sensor woken up")
        print("Sensor woken up.")

    def on_reporting_enabled_ack(self):
        """Called when proactive reporting enabled acknowledgment is received (0x19)"""
        self.update_status("Streaming IMU data...")
        print("Proactive reporting enabled.")

    def on_ble_stay_connected_ack(self):
        """Called when BLE stay connected acknowledgment is received (0x29)"""
        print("BLE stay connected enabled.")

    def on_ble_highspeed_ack(self):
        """Called when BLE high-speed communication acknowledgment is received (0x46)"""
        print("BLE high-speed communication enabled.")

    def on_packet_header(self, tag, timestamp_ms):
        """Called when packet header is parsed"""
        print(f"\nSubscribe tag: 0x{tag:04x}, ms: {timestamp_ms}")

    def on_accel_no_gravity(self, ax, ay, az):
        """Called when acceleration data without gravity is received"""
        print(f"\taX: {ax:.3f}, aY: {ay:.3f}, aZ: {az:.3f}")

    def on_accel_with_gravity(self, ax, ay, az, magnitude):
        """Called when acceleration data with gravity is received"""
        print(f"\tAX: {ax:.3f}, AY: {ay:.3f}, AZ: {az:.3f}, Mag: {magnitude:.3f}")

    def on_gyroscope(self, gx, gy, gz):
        """Called when gyroscope data is received"""
        print(f"\tGX: {gx:.3f}, GY: {gy:.3f}, GZ: {gz:.3f}")

    def on_magnetometer(self, cx, cy, cz, magnitude):
        """Called when magnetometer data is received"""
        print(f"\tCX: {cx:.3f}, CY: {cy:.3f}, CZ: {cz:.3f}, Mag: {magnitude:.3f}")

    def on_environmental(self, temperature, air_pressure, height):
        """Called when environmental data is received"""
        print(f"\tTemp: {temperature:.2f}°C, Pressure: {air_pressure:.3f}, Height: {height:.3f}")

    def on_quaternion(self, w, x, y, z):
        """Called when quaternion data is received - updates visualization (no gimbal lock!)"""
        print(f"\tw: {w:.3f}, x: {x:.3f}, y: {y:.3f}, z: {z:.3f}")
        if self.visualizer:
            # Schedule UI update on main thread using quaternions
            Clock.schedule_once(lambda dt: self.visualizer.set_quaternion(w, x, y, z), 0)

    def on_euler_angles(self, roll_val, pitch_val, yaw_val):
        """Called when Euler angle data is received - just for display"""
        print(f"\tRoll: {roll_val:.3f}°, Pitch: {pitch_val:.3f}°, Yaw: {yaw_val:.3f}°")
        
        # Update global orientation variables for display
        self.roll = roll_val
        self.pitch = pitch_val
        self.yaw = yaw_val
        
        # Update the text display on the main thread
        Clock.schedule_once(lambda dt: self.update_orientation_display())

    def on_unknown_command(self, command_id):
        """Called when unknown command ID is received"""
        print(f"Error! Command ID not defined: 0x{command_id:02X}")

    def notification_handler(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        """Called when BLE notification data is received"""
        # BLE delivers the payload directly (no serial packet framing needed)
        # Call Cmd_RxUnpack directly with the data (like the working example)
        Cmd_RxUnpack(data, len(data), callbacks=self.imu_callbacks)

    def run_ble_in_thread(self):
        """Run BLE connection in a separate thread with its own event loop"""
        # Create a new event loop for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            loop.run_until_complete(self.ble_task())
        except Exception as e:
            print(f"BLE Error: {e}")
            self.update_status(f"Error: {e}")
            Clock.schedule_once(lambda dt: setattr(self.start_button, 'disabled', False), 0)
            Clock.schedule_once(lambda dt: setattr(self.start_button, 'text', 'Connect to IMU'), 0)
        finally:
            loop.close()

    async def ble_task(self):
        """Main BLE connection and communication task"""
        self.update_status("Scanning for IMU device...")
        print("Starting scan...")
        
        # Use the same scanner parameters as the working example
        device = await BleakScanner.find_device_by_address(
            par_device_addr, cb=dict(use_bdaddr=False)
        )

        if device is None:
            self.update_status(f"Device {par_device_addr} not found!")
            Clock.schedule_once(lambda dt: setattr(self.start_button, 'disabled', False), 0)
            Clock.schedule_once(lambda dt: setattr(self.start_button, 'text', 'Connect to IMU'), 0)
            print(f"Could not find device with address {par_device_addr}")
            return

        self.update_status("Connecting to device...")
        print("Connecting to device...")
        
        # Event for disconnect handling
        disconnected_event = asyncio.Event()

        def disconnected_callback(client):
            print("Disconnected callback called!")
            self.update_status("Device disconnected!")
            disconnected_event.set()
            Clock.schedule_once(lambda dt: setattr(self.start_button, 'disabled', False), 0)
            Clock.schedule_once(lambda dt: setattr(self.start_button, 'text', 'Connect to IMU'), 0)

        async with BleakClient(device, disconnected_callback=disconnected_callback) as client:
            self.client = client
            self.update_status("Connected! Initializing...")
            Clock.schedule_once(lambda dt: setattr(self.start_button, 'text', 'Connected'), 0)
            print("Connected")
            
            # Start notifications
            await client.start_notify(par_notification_characteristic, self.notification_handler)
            
            # Send initialization commands
            await client.write_gatt_char(par_write_characteristic, bytes([0x29]))
            await client.write_gatt_char(par_write_characteristic, bytes([0x46]))

            # Configure IMU parameters
            isCompassOn = 0  # Whether to use magnetic field fusion
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
            await client.write_gatt_char(par_write_characteristic, params)

            # Enable proactive reporting
            await client.write_gatt_char(par_write_characteristic, bytes([0x19]))

            self.update_status("Streaming IMU data...")
            print("Receiving data...")

            # Wait until disconnected
            await disconnected_event.wait()

        self.update_status("Disconnected.")
        print("Disconnected")

    def on_start_ble(self, instance):
        """Called when the start button is pressed"""
        self.start_button.disabled = True
        self.start_button.text = "Connecting..."
        self.update_status("Launching BLE connection...")

        # Run BLE in a separate thread to avoid blocking Kivy's main loop
        ble_thread = threading.Thread(target=self.run_ble_in_thread, daemon=True)
        ble_thread.start()


class BLEIMUApp(App):
    def build(self):
        return BLEWidget()


if __name__ == "__main__":
    BLEIMUApp().run()