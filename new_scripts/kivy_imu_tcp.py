from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.graphics import Color, Quad
import socket
import threading

# TCP connection settings
TCP_HOST = "192.168.0.102"  # ESP32 IP address - adjust as needed
TCP_PORT = 25


class IMU3DVisualizer(Widget):
    """Widget that displays a 3D representation of the IMU orientation"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.quat_w = 1.0
        self.quat_x = 0.0
        self.quat_y = 0.0
        self.quat_z = 0.0
        self.bind(pos=self.update_canvas, size=self.update_canvas)
        self.update_canvas()
    
    def set_quaternion(self, w, x, y, z):
        """Update orientation using quaternion (avoids gimbal lock!)"""
        # Correct IMU to OpenGL coordinate system mapping
        self.quat_w, self.quat_x, self.quat_y, self.quat_z = w, -x, z, y
        self.update_canvas()
    
    def _rotate_point_quat(self, x, y, z):
        """Rotate point using quaternion (no gimbal lock!)"""
        w, qx, qy, qz = self.quat_w, self.quat_x, self.quat_y, self.quat_z
        
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
    
    def _project_3d_to_2d(self, x, y, z, scale, cx, cy):
        """Simple orthographic projection"""
        return cx + x * scale, cy + y * scale, z
    
    def update_canvas(self, *args):
        """Redraw the 3D visualization"""
        self.canvas.clear()
        
        cx = self.center_x
        cy = self.center_y
        scale = min(self.width, self.height) / 2.5
        
        # Define box vertices (matching OpenGL version)
        vertices = [
            ( 1.0,  0.2, -1.0), (-1.0,  0.2, -1.0), (-1.0,  0.2,  1.0), ( 1.0,  0.2,  1.0),
            ( 1.0, -0.2,  1.0), (-1.0, -0.2,  1.0), (-1.0, -0.2, -1.0), ( 1.0, -0.2, -1.0),
            ( 1.0,  0.2,  1.0), (-1.0,  0.2,  1.0), (-1.0, -0.2,  1.0), ( 1.0, -0.2,  1.0),
            ( 1.0, -0.2, -1.0), (-1.0, -0.2, -1.0), (-1.0,  0.2, -1.0), ( 1.0,  0.2, -1.0),
            (-1.0,  0.2,  1.0), (-1.0,  0.2, -1.0), (-1.0, -0.2, -1.0), (-1.0, -0.2,  1.0),
            ( 1.0,  0.2, -1.0), ( 1.0,  0.2,  1.0), ( 1.0, -0.2,  1.0), ( 1.0, -0.2, -1.0),
        ]
        
        colors = [
            (0.0, 1.0, 0.0),  # Top - green
            (1.0, 0.5, 0.0),  # Bottom - orange
            (1.0, 0.0, 0.0),  # Front - red
            (1.0, 1.0, 0.0),  # Back - yellow
            (0.0, 0.0, 1.0),  # Left - blue
            (1.0, 0.0, 1.0),  # Right - magenta
        ]
        
        projected_faces = []
        for face_idx in range(6):
            face_vertices = []
            for vert_idx in range(4):
                v = vertices[face_idx * 4 + vert_idx]
                rx, ry, rz = self._rotate_point_quat(v[0], v[1], v[2])
                px, py, pz = self._project_3d_to_2d(rx, ry, rz, scale, cx, cy)
                face_vertices.append((px, py, pz))
            avg_z = sum(v[2] for v in face_vertices) / 4
            projected_faces.append((avg_z, face_idx, face_vertices))
        
        projected_faces.sort(key=lambda f: f[0])
        
        with self.canvas:
            for avg_z, face_idx, face_verts in projected_faces:
                color = colors[face_idx]
                Color(color[0], color[1], color[2], 1.0)
                Quad(points=[
                    face_verts[0][0], face_verts[0][1],
                    face_verts[1][0], face_verts[1][1],
                    face_verts[2][0], face_verts[2][1],
                    face_verts[3][0], face_verts[3][1],
                ])


class TCPWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'vertical'
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.sock = None
        self.running = False
        
        # Status label
        self.status_label = Label(
            text=f"Press 'Connect' to start ({TCP_HOST}:{TCP_PORT})",
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

    def update_status(self, text):
        Clock.schedule_once(lambda dt: setattr(self.status_label, 'text', text))

    def update_orientation_display(self):
        text = f"roll: {self.roll:.2f}, pitch: {self.pitch:.2f}, yaw: {self.yaw:.2f}"
        Clock.schedule_once(lambda dt: setattr(self.orientation_label, 'text', text))

    def parse_imu_line(self, line):
        """
        Parse CSV line from ESP32 TCP server.
        Format: ts,ang[3],awg[3],gyro[3],mag[3],quat[4],euler[3],temp,press,height
        Indices: 0, 1-3,   4-6,   7-9,    10-12, 13-16,  17-19,   20,  21,   22
        """
        parts = line.strip().split(',')
        if len(parts) < 20:
            return False
        
        try:
            # Quaternion at indices 13-16 (w, x, y, z)
            w = float(parts[13])
            x = float(parts[14])
            y = float(parts[15])
            z = float(parts[16])
            
            # Euler angles at indices 17-19 (roll, pitch, yaw)
            self.roll = float(parts[17])
            self.pitch = float(parts[18])
            self.yaw = float(parts[19])
            
            # Update UI on main thread
            Clock.schedule_once(lambda dt: self.visualizer.set_quaternion(w, x, y, z), 0)
            Clock.schedule_once(lambda dt: self.update_orientation_display(), 0)
            
            return True
        except:
            return False

    def tcp_receiver_thread(self):
        """TCP receiver running in separate thread"""
        buf = ""
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((TCP_HOST, TCP_PORT))
            self.sock.settimeout(0.5)
            
            self.update_status("Connected! Streaming data...")
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'text', 'Disconnect'), 0)
            
            while self.running:
                try:
                    data = self.sock.recv(1024).decode(errors='ignore')
                except socket.timeout:
                    continue
                except:
                    break
                
                if not data:
                    break
                
                buf += data
                
                while "\n" in buf:
                    line, _, buf = buf.partition("\n")
                    self.parse_imu_line(line)
                    
        except socket.error as e:
            self.update_status(f"Connection failed: {e}")
        finally:
            if self.sock:
                self.sock.close()
                self.sock = None
            self.running = False
            self.update_status(f"Disconnected ({TCP_HOST}:{TCP_PORT})")
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'text', 'Connect'), 0)
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'disabled', False), 0)

    def on_connect(self, instance):
        if self.running:
            # Disconnect
            self.running = False
            self.connect_button.disabled = True
        else:
            # Connect
            self.running = True
            self.connect_button.text = "Connecting..."
            self.update_status(f"Connecting to {TCP_HOST}:{TCP_PORT}...")
            
            thread = threading.Thread(target=self.tcp_receiver_thread, daemon=True)
            thread.start()


class IMUTCPApp(App):
    def build(self):
        return TCPWidget()


if __name__ == "__main__":
    IMUTCPApp().run()