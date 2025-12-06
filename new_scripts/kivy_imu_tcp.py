import sys
import socket
import threading
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.graphics import Color, Mesh

# TCP connection settings
TCP_HOST = "192.168.0.105"  # ESP32 IP address - adjust as needed
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
        
        # Define faces with outward-facing normals (CCW winding when viewed from outside)
        # Each face: [v0, v1, v2, v3] in CCW order when looking at the face from outside
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
        # Edge vectors
        e1 = (v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2])
        e2 = (v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2])
        
        # Cross product
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
            from kivy.graphics import Rectangle
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


class TCPWidget(BoxLayout):
    def __init__(self, host, port, **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'vertical'
        
        self.tcp_host = host
        self.tcp_port = port
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.sock = None
        self.running = False
        
        # Status label
        self.status_label = Label(
            text=f"Press 'Connect' to start ({self.tcp_host}:{self.tcp_port})",
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
            Clock.schedule_once(lambda dt, w=w, x=x, y=y, z=z: 
                                self.visualizer.set_quaternion(w, x, y, z), 0)
            Clock.schedule_once(lambda dt: self.update_orientation_display(), 0)
            
            return True
        except (ValueError, IndexError):
            return False

    def tcp_receiver_thread(self):
        """TCP receiver running in separate thread"""
        buf = ""
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.tcp_host, self.tcp_port))
            self.sock.settimeout(0.5)
            
            self.update_status("Connected! Streaming data...")
            Clock.schedule_once(lambda dt: setattr(self.connect_button, 'text', 'Disconnect'), 0)
            
            while self.running:
                try:
                    data = self.sock.recv(1024).decode(errors='ignore')
                except socket.timeout:
                    continue
                except socket.error:
                    break
                
                if not data:
                    break
                
                buf += data
                
                # Process complete lines using partition
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
            self.update_status(f"Disconnected ({self.tcp_host}:{self.tcp_port})")
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
            self.update_status(f"Connecting to {self.tcp_host}:{self.tcp_port}...")
            
            thread = threading.Thread(target=self.tcp_receiver_thread, daemon=True)
            thread.start()


class IMUTCPApp(App):
    def __init__(self, host, port, **kwargs):
        super().__init__(**kwargs)
        self.host = host
        self.port = port
    
    def build(self):
        self.title = "IMU Visualization (TCP) - Kivy"
        return TCPWidget(self.host, self.port)


if __name__ == "__main__":
    # Allow command line override of host
    host = TCP_HOST
    port = TCP_PORT
    
    if len(sys.argv) > 1:
        host = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            port = int(sys.argv[2])
        except ValueError:
            pass
    
    print(f"IMU Visualization - Target: {host}:{port}")
    print("Press 'Connect' button to start streaming\n")
    
    IMUTCPApp(host, port).run()