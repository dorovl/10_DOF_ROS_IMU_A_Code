import asyncio
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.graphics import Color, Rectangle, Rotate, PushMatrix, PopMatrix, Translate
from kivy.graphics.opengl import glEnable, glDisable, GL_DEPTH_TEST
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
import math

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
        self.bind(pos=self.update_canvas, size=self.update_canvas)
        self.update_canvas()
    
    def set_orientation(self, roll, pitch, yaw):
        """Update the orientation angles and redraw"""
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.update_canvas()
    
    def update_canvas(self, *args):
        """Redraw the 3D visualization"""
        self.canvas.clear()
        
        with self.canvas:
            # Enable depth testing for 3D effect
            PushMatrix()
            
            # Center the visualization
            cx = self.center_x
            cy = self.center_y
            
            # Scale for the IMU box
            scale = min(self.width, self.height) / 8
            
            # Apply transformations: translate to center
            Translate(cx, cy, 0)
            
            # Apply rotations (order matters: yaw, pitch, roll)
            Rotate(angle=self.yaw, axis=(0, 0, 1))      # Yaw around Z
            Rotate(angle=-self.pitch, axis=(1, 0, 0))   # Pitch around X
            Rotate(angle=-self.roll, axis=(0, 1, 0))    # Roll around Y
            
            # Draw the IMU box with 6 colored faces
            self._draw_box(scale)
            
            PopMatrix()
    
    def _draw_box(self, scale):
        """Draw a colored 3D box representing the IMU"""
        w = scale * 2
        h = scale * 0.4
        d = scale * 1.5
        
        # Top face (green)
        Color(0.0, 1.0, 0.0, 1.0)
        Rectangle(pos=(-w/2, h/2), size=(w, d))
        
        # Bottom face (orange)
        Color(1.0, 0.5, 0.0, 1.0)
        Rectangle(pos=(-w/2, -h/2 - d), size=(w, d))
        
        # Front face (red)
        Color(1.0, 0.0, 0.0, 1.0)
        Rectangle(pos=(-w/2, -h/2), size=(w, h))
        
        # Back face (yellow)
        Color(1.0, 1.0, 0.0, 1.0)
        Rectangle(pos=(-w/2, h/2), size=(w, h))
        
        # Left face (blue)
        Color(0.0, 0.0, 1.0, 1.0)
        Rectangle(pos=(-w/2, -h/2), size=(d * 0.3, h))
        
        # Right face (magenta)
        Color(1.0, 0.0, 1.0, 1.0)
        Rectangle(pos=(w/2 - d * 0.3, -h/2), size=(d * 0.3, h))


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
        self.disconnected_event = asyncio.Event()
        
        # Setup IMU callbacks
        self.imu_callbacks = {
            'config_ack': self.on_config_ack,
            'wakeup_ack': self.on_wakeup_ack,
            'reporting_enabled_ack': self.on_reporting_enabled_ack,
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
        self.update_status("✓ Configuration set successfully")
        print("Configuration set successfully.")

    def on_wakeup_ack(self):
        """Called when sensor wake-up acknowledgment is received (0x03)"""
        self.update_status("✓ Sensor woken up")
        print("Sensor woken up.")

    def on_reporting_enabled_ack(self):
        """Called when proactive reporting enabled acknowledgment is received (0x19)"""
        self.update_status("✓ Streaming IMU data...")
        print("Proactive reporting enabled.")

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
        """Called when quaternion data is received"""
        print(f"\tw: {w:.3f}, x: {x:.3f}, y: {y:.3f}, z: {z:.3f}")

    def on_euler_angles(self, roll_val, pitch_val, yaw_val):
        """Called when Euler angle data is received - updates visualization"""
        print(f"\tRoll: {roll_val:.3f}°, Pitch: {pitch_val:.3f}°, Yaw: {yaw_val:.3f}°")
        
        # Update global orientation variables
        self.roll = roll_val
        self.pitch = pitch_val
        self.yaw = yaw_val
        
        # Update the display and visualization on the main thread
        Clock.schedule_once(lambda dt: self.update_orientation_display())
        Clock.schedule_once(lambda dt: self.visualizer.set_orientation(self.roll, self.pitch, self.yaw))

    def on_unknown_command(self, command_id):
        """Called when unknown command ID is received"""
        print(f"Error! Command ID not defined: 0x{command_id:02X}")

    def notification_handler(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        """Called when BLE notification data is received"""
        # BLE delivers the payload directly (no serial packet framing needed)
        # Call Cmd_RxUnpack directly with the data (like the working example)
        Cmd_RxUnpack(data, len(data), callbacks=self.imu_callbacks)

    async def ble_task(self):
        """Main BLE connection and communication task"""
        self.update_status("🔍 Scanning for IMU device...")
        
        # Use the same scanner parameters as the working example
        device = await BleakScanner.find_device_by_address(
            par_device_addr, cb=dict(use_bdaddr=False)
        )

        if device is None:
            self.update_status(f"❌ Device {par_device_addr} not found!")
            Clock.schedule_once(lambda dt: setattr(self.start_button, 'disabled', False), 0)
            return

        self.update_status("📡 Connecting to device...")
        self.disconnected_event.clear()

        def disconnected_callback(client):
            self.update_status("❌ Device disconnected!")
            self.disconnected_event.set()
            Clock.schedule_once(lambda dt: setattr(self.start_button, 'disabled', False), 0)

        async with BleakClient(device, disconnected_callback=disconnected_callback) as client:
            self.client = client
            self.update_status("✓ Connected! Initializing...")
            
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

            self.update_status("✓ Streaming IMU data...")

            # Wait until disconnected
            await self.disconnected_event.wait()

        self.update_status("Disconnected.")

    def on_start_ble(self, instance):
        """Called when the start button is pressed"""
        self.start_button.disabled = True
        self.start_button.text = "Connecting..."
        self.update_status("Launching BLE connection...")

        # Run BLE task without blocking Kivy main thread
        asyncio.ensure_future(self.ble_task())


class BLEIMUApp(App):
    def build(self):
        return BLEWidget()


if __name__ == "__main__":
    BLEIMUApp().run()