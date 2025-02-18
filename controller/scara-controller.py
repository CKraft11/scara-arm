import sys
import math
import csv 
import win32gui
import win32con
import win32process
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLabel, QLineEdit, QPushButton, QListWidget, 
                           QListWidgetItem, QGridLayout, QFileDialog, 
                           QCheckBox, QTabWidget, QOpenGLWidget)
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QImage
from PyQt5.QtCore import Qt, QTimer, QRect, QPoint
from ctypes import windll
import pyqtgraph as pg
import numpy as np
import pkgutil
import time
import subprocess
import pybullet as p
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import multiprocessing
from multiprocessing import Process, Value, Array
pg.setConfigOptions(antialias=True)

# SCARA robot dimensions
L1 = 180  # Length of the first linkage (mm)
L2 = 180  # Length of the second linkage (mm)
THETA1_LIMIT = 110  #Angular Limit of Theta 1 angle to prevent crashing (degrees)
THETA2_LIMIT = 160  #Angular Limit of Theta 2 angle to prevent crashing (degrees)
Z_MIN = 0  # Minimum Z value
Z_MAX = 80  # Maximum Z value

GRAPH_WIDTH = 700 
SIMULATION_WIDTH = 800
SIMULATION_OFFSET = SIMULATION_WIDTH-700
WINDOW_WIDTH = GRAPH_WIDTH + SIMULATION_WIDTH  # Will be dynamically calculated
WINDOW_HEIGHT = 800

# Colors
BLACK = QColor(0, 0, 0)
WHITE = QColor(255, 255, 255)
RED = QColor(255, 0, 0)
GREEN = QColor(0, 255, 0)
BLUE = QColor(0, 0, 255)
GRAY = QColor(128, 128, 128)
BUTTON_COLOR = QColor(200, 200, 200)

# Scale factor for displaying the robot
SCALE = 1

# List to store waypoints
waypoints = []

class JointStepper:
    def __init__(self):
        self.current_position = 0
        self.target_position = 0
        self.current_speed = 0
        self.base_max_speed = 72  # Base maximum speed (degrees per second)
        self.base_acceleration = 72  # Base acceleration (degrees per second²)
        self.max_speed = self.base_max_speed
        self.acceleration = self.base_acceleration
        self.last_update_time = time.time()
        self.move_duration = 0
        
        # Motion smoothing buffer
        self.position_buffer = []
        self.velocity_buffer = []
        self.time_buffer = []
        self.buffer_size = 5
        
    def set_target(self, target_position: float, move_duration: float = None):
        self.target_position = target_position
        if move_duration is not None:
            self.move_duration = move_duration
            distance = abs(self.target_position - self.current_position)
            if distance > 0 and move_duration > 0:
                half_time = move_duration / 2
                self.acceleration = (2 * distance) / (half_time * half_time)
                self.max_speed = self.acceleration * half_time
                
                speed_factor = min(1.0, self.base_max_speed / self.max_speed)
                self.max_speed *= speed_factor
                self.acceleration *= speed_factor
    
    def get_smoothed_velocity(self) -> float:
        """Calculate smoothed velocity from position buffer"""
        if len(self.position_buffer) < 2:
            return 0.0
            
        # Use last few points to calculate average velocity
        positions = self.position_buffer[-self.buffer_size:]
        times = self.time_buffer[-self.buffer_size:]
        
        if len(positions) < 2:
            return 0.0
            
        # Calculate velocities between consecutive points
        velocities = []
        for i in range(1, len(positions)):
            dt = times[i] - times[i-1]
            if dt > 0:
                v = (positions[i] - positions[i-1]) / dt
                velocities.append(v)
        
        if not velocities:
            return 0.0
            
        # Apply weighted average (more recent values have higher weight)
        weights = [i/sum(range(1, len(velocities) + 1)) for i in range(1, len(velocities) + 1)]
        return sum(v * w for v, w in zip(velocities, weights))
    
    def update(self) -> tuple[float, float]:
        """Update motion and return current position and smoothed velocity"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        # Ensure minimum dt to prevent extreme values during stutters
        dt = min(dt, 0.1)
        
        # Calculate distance to target
        distance = self.target_position - self.current_position
        direction = 1 if distance > 0 else -1 if distance < 0 else 0

        if direction == 0:
            if abs(self.current_speed) > 0:
                decel = self.acceleration * dt
                self.current_speed = max(0, abs(self.current_speed) - decel) * (1 if self.current_speed > 0 else -1)
        else:
            stopping_dist = (self.current_speed * self.current_speed) / (2 * self.acceleration)
            
            if abs(distance) > stopping_dist:
                self.current_speed += direction * self.acceleration * dt
                self.current_speed = min(max(self.current_speed, -self.max_speed), self.max_speed)
            else:
                decel = self.acceleration * dt
                self.current_speed = max(0, abs(self.current_speed) - decel) * direction

        # Update position
        self.current_position += self.current_speed * dt
        
        # Clamp to target if overshot
        if (direction > 0 and self.current_position > self.target_position) or \
           (direction < 0 and self.current_position < self.target_position):
            self.current_position = self.target_position
            self.current_speed = 0
        
        # Update buffers
        self.position_buffer.append(self.current_position)
        self.time_buffer.append(current_time)
        
        # Trim buffers
        if len(self.position_buffer) > self.buffer_size:
            self.position_buffer.pop(0)
            self.time_buffer.pop(0)
        
        self.last_update_time = current_time
        
        # Return both position and smoothed velocity
        return self.current_position, self.get_smoothed_velocity()

class SCARAMotionController:
    def __init__(self):
        self.shoulder = JointStepper()
        self.elbow = JointStepper()
        self.z_axis = JointStepper()
        self.end_effector = JointStepper()

    def set_target(self, theta1: float, theta2: float, z: float, rotation: float):
        """Set target positions for all joints with synchronized timing"""
        shoulder_dist = abs(theta1 - self.shoulder.current_position)
        elbow_dist = abs(theta2 - self.elbow.current_position)
        max_dist = max(shoulder_dist, elbow_dist)
        
        if max_dist > 0:
            # Calculate move duration based on the longest movement
            accel_time = self.shoulder.base_max_speed / self.shoulder.base_acceleration
            accel_dist = 0.5 * self.shoulder.base_acceleration * accel_time * accel_time
            
            if max_dist <= 2 * accel_dist:
                move_duration = 2 * math.sqrt(max_dist / self.shoulder.base_acceleration)
            else:
                constant_velocity_dist = max_dist - 2 * accel_dist
                constant_velocity_time = constant_velocity_dist / self.shoulder.base_max_speed
                move_duration = 2 * accel_time + constant_velocity_time
            
            # Scale speeds based on relative distances
            shoulder_speed_scale = shoulder_dist / max_dist if max_dist > 0 else 1
            elbow_speed_scale = elbow_dist / max_dist if max_dist > 0 else 1
            
            # Set targets with scaled speeds
            self.shoulder.set_target(theta1, move_duration)
            self.elbow.set_target(theta2, move_duration)
            
            # Apply speed scaling to each joint
            self.shoulder.acceleration *= shoulder_speed_scale
            self.elbow.acceleration *= elbow_speed_scale
        else:
            move_duration = 0.1
            self.shoulder.set_target(theta1, move_duration)
            self.elbow.set_target(theta2, move_duration)
        
        self.z_axis.set_target(z)
        self.end_effector.set_target(rotation)

    def update(self) -> tuple[float, float, float, float, float, float]:
        """Update all joints and return current positions and velocities"""
        theta1, vel1 = self.shoulder.update()
        theta2, vel2 = self.elbow.update()
        z, _ = self.z_axis.update()
        rotation, _ = self.end_effector.update()
        
        return theta1, theta2, z, rotation, vel1, vel2  # Now returns velocities too

    def is_at_target(self, threshold: float = 0.1) -> bool:
        """Check if all joints are at their targets"""
        return (abs(self.shoulder.current_position - self.shoulder.target_position) < threshold and
                abs(self.elbow.current_position - self.elbow.target_position) < threshold and
                abs(self.z_axis.current_position - self.z_axis.target_position) < threshold and
                abs(self.end_effector.current_position - self.end_effector.target_position) < threshold)
    

class AngleGraphWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(GRAPH_WIDTH, WINDOW_HEIGHT)
        
        # Main layout
        layout = QVBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create angle plot
        self.angle_plot = pg.PlotWidget(title="Joint Angles")
        self.angle_plot.setLabel('left', 'Angle', units='degrees')
        self.angle_plot.setLabel('bottom', 'Time', units='s')
        self.angle_plot.setYRange(-180, 180)
        self.angle_plot.showGrid(x=True, y=True)
        
        # Create velocity plot
        self.velocity_plot = pg.PlotWidget(title="Joint Velocities")
        self.velocity_plot.setLabel('left', 'Velocity', units='deg/s')
        self.velocity_plot.setLabel('bottom', 'Time', units='s')
        self.velocity_plot.setYRange(-200, 200)
        self.velocity_plot.showGrid(x=True, y=True)
        
        # Set dark theme
        self.angle_plot.setBackground('k')
        self.velocity_plot.setBackground('k')
        
        # Create plot lines with different colors and names
        self.theta1_line = self.angle_plot.plot(pen=pg.mkPen('b', width=2))
        self.theta2_line = self.angle_plot.plot(pen=pg.mkPen('g', width=2))
        self.theta3_line = self.angle_plot.plot(pen=pg.mkPen('r', width=2))
        self.vel1_line = self.velocity_plot.plot(pen=pg.mkPen('b', width=2))
        self.vel2_line = self.velocity_plot.plot(pen=pg.mkPen('g', width=2))
        
        # Add legends
        angle_legend = pg.LegendItem(offset=(50,10))
        angle_legend.setParentItem(self.angle_plot.graphicsItem())
        angle_legend.addItem(self.theta1_line, "Theta 1 (Shoulder)")
        angle_legend.addItem(self.theta2_line, "Theta 2 (Elbow)")
        angle_legend.addItem(self.theta3_line, "Theta 3 (Wrist)")
        
        velocity_legend = pg.LegendItem(offset=(50,10))
        velocity_legend.setParentItem(self.velocity_plot.graphicsItem())
        velocity_legend.addItem(self.vel1_line, "Shoulder Velocity")
        velocity_legend.addItem(self.vel2_line, "Elbow Velocity")
        
        # Add plots to layout
        layout.addWidget(self.angle_plot)
        layout.addWidget(self.velocity_plot)
        self.setLayout(layout)
        
        # Initialize data buffers
        self.time_data = []
        self.theta1_data = []
        self.theta2_data = []
        self.theta3_data = []
        self.vel1_data = []
        self.vel2_data = []
        
        # Initialize timing variables
        self.start_time = None
        self.paused_time = 0
        self.last_update = None
        self.display_time = 10  # Show last 10 seconds of data
        
        # Set up update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(50)  # 20 FPS update rate

    # 2. Add this new update_plot method:
    def update_plot(self, theta1, theta2, theta3, vel1, vel2, is_paused=False):
        """Update plots with new data including theta3"""
        current_time = time.time()
        
        if self.start_time is None:
            self.start_time = current_time
            self.last_update = current_time
            self.paused_time = 0
        
        # Calculate elapsed time
        if is_paused and self.last_update is not None:
            self.paused_time += current_time - self.last_update
            
        elapsed = current_time - self.start_time - self.paused_time
        
        # Add new data point
        self.time_data.append(elapsed)
        self.theta1_data.append(theta1)
        self.theta2_data.append(theta2)
        self.theta3_data.append(theta3)
        self.vel1_data.append(vel1)
        self.vel2_data.append(vel2)
        
        # Keep only recent data
        while len(self.time_data) > 1 and self.time_data[-1] - self.time_data[0] > self.display_time:
            self.time_data.pop(0)
            self.theta1_data.pop(0)
            self.theta2_data.pop(0)
            self.theta3_data.pop(0)
            self.vel1_data.pop(0)
            self.vel2_data.pop(0)
        
        self.last_update = current_time

    # 3. Update the update_display method:
    def update_display(self):
        """Update the plot display"""
        if not self.time_data:
            return
            
        try:
            # Ensure all arrays are the same length
            min_length = min(len(self.time_data), 
                           len(self.theta1_data), 
                           len(self.theta2_data),
                           len(self.theta3_data),
                           len(self.vel1_data),
                           len(self.vel2_data))
            
            time_data = self.time_data[-min_length:]
            theta1_data = self.theta1_data[-min_length:]
            theta2_data = self.theta2_data[-min_length:]
            theta3_data = self.theta3_data[-min_length:]
            vel1_data = self.vel1_data[-min_length:]
            vel2_data = self.vel2_data[-min_length:]
            
            # Update the plot data
            self.theta1_line.setData(time_data, theta1_data)
            self.theta2_line.setData(time_data, theta2_data)
            self.theta3_line.setData(time_data, theta3_data)
            self.vel1_line.setData(time_data, vel1_data)
            self.vel2_line.setData(time_data, vel2_data)
            
            # Update plot ranges
            current_time = time_data[-1]
            self.angle_plot.setXRange(max(0, current_time - self.display_time), current_time)
            self.velocity_plot.setXRange(max(0, current_time - self.display_time), current_time)
        except Exception as e:
            print(f"Error updating display: {e}")
            return

    # 4. Update the reset_plot method:
    def reset_plot(self):
        """Clear all plot data"""
        self.time_data = []
        self.theta1_data = []
        self.theta2_data = []
        self.theta3_data = []
        self.vel1_data = []
        self.vel2_data = []
        self.start_time = None
        self.paused_time = 0
        self.last_update = None
        
        # Clear the plots
        self.theta1_line.setData([], [])
        self.theta2_line.setData([], [])
        self.theta3_line.setData([], [])
        self.vel1_line.setData([], [])
        self.vel2_line.setData([], [])
        
class PyBulletProcess(Process):
    def __init__(self):
        super().__init__()
        # Shared memory for joint angles
        self.theta1 = Value('d', 0.0)
        self.theta2 = Value('d', 0.0)
        self.z = Value('d', 0.0)
        self.rotation = Value('d', 0.0)
        self.running = Value('i', 1)
        
    def run(self):
        import pybullet as p
        import math
        import time
        
        # Initialize PyBullet
        physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.setGravity(0, 0, 0)
        
        # Load URDF
        robot = p.loadURDF("scara_urdf/urdf/scara_urdf.urdf", [0, 0, 0], useFixedBase=1)
        
        # Configure camera view
        p.resetDebugVisualizerCamera(
            cameraDistance=0.6,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )
        
        # Main loop
        while self.running.value:
            # Get current angles from shared memory
            theta1_rad = math.radians(self.theta1.value)
            theta2_rad = math.radians(self.theta2.value)
            end_rotation = -(theta1_rad + theta2_rad + math.radians(self.rotation.value))
            
            # Update joint positions
            p.resetJointState(robot, 0, theta1_rad)
            p.resetJointState(robot, 1, theta2_rad)
            p.resetJointState(robot, 2, -self.z.value / 1000.0)
            p.resetJointState(robot, 3, end_rotation)
            
            # Step simulation
            p.stepSimulation()
            time.sleep(0.016)  # 60 FPS
            
        # Cleanup
        p.disconnect()

class TabbedSimulationWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(SIMULATION_WIDTH, WINDOW_HEIGHT)
        self.theta1 = 0
        self.theta2 = 0
        self.z = 0
        self.rotation = 0
        
        # Initialize motion controller
        self.motion_controller = SCARAMotionController()
        
        # Create tab widget
        self.tab_widget = QTabWidget(self)
        self.tab_widget.setFixedSize(SIMULATION_WIDTH, WINDOW_HEIGHT)
        
        # Create 2D view tab
        self.view_2d = SimulationWidget2D(self)
        self.tab_widget.addTab(self.view_2d, "2D View")
        
        # Create 3D view tab
        self.view_3d = SimulationWidget3D(self)
        self.tab_widget.addTab(self.view_3d, "3D View")
        
        # Set up layout
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.tab_widget)
        self.setLayout(layout)
        
        # Timer for updating views
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_simulation)
        self.update_timer.start(16)  # 60 FPS

    def update_simulation(self):
        """Update both views"""
        # Update positions
        self.view_2d.theta1 = self.theta1
        self.view_2d.theta2 = self.theta2
        self.view_2d.z = self.z
        self.view_2d.rotation = self.rotation
        
        self.view_3d.theta1 = self.theta1
        self.view_3d.theta2 = self.theta2
        self.view_3d.z = self.z
        self.view_3d.rotation = self.rotation
        
        # Update current view
        current_index = self.tab_widget.currentIndex()
        if current_index == 0:
            self.view_2d.update()

    def closeEvent(self, event):
        """Handle cleanup when closing"""
        self.update_timer.stop()
        self.view_3d.closeEvent(event)
        super().closeEvent(event)  # 60 FPS
class SimulationWidget2D(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.theta1 = 0
        self.theta2 = 0
        self.z = 0
        self.rotation = 0
        self.angle = 0
        self.setFixedSize(SIMULATION_WIDTH, WINDOW_HEIGHT)
        
        self.setAttribute(Qt.WA_OpaquePaintEvent)
        self.setAttribute(Qt.WA_NoSystemBackground)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.HighQualityAntialiasing)

        # Draw the background
        painter.fillRect(self.rect(), BLACK)

        # Draw the workspace limits first
        self.draw_workspace_limits(painter)
        
        # Draw the robot
        base, joint1, end_effector = self.draw_robot(painter)
        
        # Draw waypoints
        self.draw_waypoints(painter, draw_paths=True)

        # Draw Z-axis
        self.draw_z_axis(painter)

        # Draw end-effector rotation last
        self.draw_end_effector_rotation(painter, end_effector)

    def closeEvent(self, event):
        """Handle PyBullet cleanup when closing"""
        try:
            p.disconnect()
        except:
            pass
        super().closeEvent(event)
    
    def draw_robot(self, painter):
        # Convert angles to radians
        theta1_rad = math.radians(self.theta1)
        theta2_rad = math.radians(self.theta2)

        # Calculate end-effector position
        x = L1 * math.cos(theta1_rad) + L2 * math.cos(theta1_rad + theta2_rad)
        y = L1 * math.sin(theta1_rad) + L2 * math.sin(theta1_rad + theta2_rad)
        theta = math.atan2(y, x)

        # Draw the robot links
        base = QPoint((SIMULATION_WIDTH // 2) -100, WINDOW_HEIGHT // 2)
        joint1 = QPoint(base.x() + int(L1 * SCALE * math.cos(theta1_rad)), 
                       base.y() + int(L1 * SCALE * math.sin(theta1_rad)))
        end_effector = QPoint(base.x() + int(x * SCALE), base.y() + int(y * SCALE))

        painter.setPen(QPen(WHITE, 5))
        painter.drawLine(base, joint1)
        painter.drawLine(joint1, end_effector)

        # Draw circles at joints and end-effector
        painter.setBrush(WHITE)
        painter.drawEllipse(base, 10, 10)
        painter.setBrush(BLUE)
        painter.drawEllipse(end_effector, 10, 10)

        return base, joint1, end_effector
    
    def draw_workspace_limits(self, painter):
        # Draw a circle representing the workspace limits
        workspace_radius = (L1 + L2) * SCALE
        workspace_center = QPoint((SIMULATION_WIDTH // 2)-SIMULATION_OFFSET, WINDOW_HEIGHT // 2)
        painter.setPen(QPen(GRAY, 2))
        painter.drawArc(QRect(workspace_center.x() - (L2 + L1), workspace_center.y() - (L2 + L1), 2 * L2 + 2 * L1, 2 * L2 + 2 * L1), int(math.radians(-THETA1_LIMIT) * 16 * 180 / math.pi), int(math.radians(2*THETA1_LIMIT) * 16 * 180 / math.pi))

        # Draw two lines representing the joint1 rotation limits
        joint1_limit_x1 = workspace_center.x() + int(L1 * SCALE * math.cos(math.radians(-THETA1_LIMIT)))
        joint1_limit_y1 = workspace_center.y() + int(L1 * SCALE * math.sin(math.radians(-THETA1_LIMIT)))
        joint1_limit_x2 = workspace_center.x() + int(L1 * SCALE * math.cos(math.radians(THETA1_LIMIT)))
        joint1_limit_y2 = workspace_center.y() + int(L1 * SCALE * math.sin(math.radians(THETA1_LIMIT)))
        painter.drawLine(workspace_center, QPoint(joint1_limit_x1, joint1_limit_y1))
        painter.drawLine(workspace_center, QPoint(joint1_limit_x2, joint1_limit_y2))
        painter.drawLine(QPoint(joint1_limit_x1, joint1_limit_y1), QPoint(joint1_limit_x1 - L2, joint1_limit_y1))
        painter.drawLine(QPoint(joint1_limit_x2, joint1_limit_y2), QPoint(joint1_limit_x2 - L2, joint1_limit_y2))
        painter.drawArc(QRect(workspace_center.x() - L2 + int(L2 * SCALE * math.cos(math.radians(-THETA1_LIMIT))), workspace_center.y() - L2 + int(L2 * SCALE * math.sin(math.radians(-THETA1_LIMIT)))-1, 2 * L2, 2 * L2), int(math.radians(THETA1_LIMIT) * 16 * 180 / math.pi), int(math.radians(180-THETA1_LIMIT) * 16 * 180 / math.pi))
        painter.drawArc(QRect(workspace_center.x() - L2 + int(L2 * SCALE * math.cos(math.radians(THETA1_LIMIT))), workspace_center.y() - L2 + int(L2 * SCALE * math.sin(math.radians(THETA1_LIMIT)))+1, 2 * L2, 2 * L2), int(math.radians(-THETA1_LIMIT) * 16 * 180 / math.pi), int(math.radians(THETA1_LIMIT-180) * 16 * 180 / math.pi))

    def draw_waypoints(self, painter, draw_paths=True):
        for i in range(len(waypoints)):
            # Update unpacking to include linear_path
            x, y, z, rotation, Wtheta1, Wtheta2, stop_at_point, duration, linear_path = waypoints[i]
            end_effector = QPoint((SIMULATION_WIDTH // 2)-SIMULATION_OFFSET + int(x), WINDOW_HEIGHT // 2 - int(y))

            # Draw different markers for stop points vs pass-through points
            painter.setBrush(RED)
            if stop_at_point:
                # Draw a square for stop points
                painter.setBrush(RED)
                painter.drawEllipse(end_effector, 5, 5)
            else:
                # Draw a circle for pass-through points
                painter.setBrush(GREEN)
                painter.drawEllipse(end_effector, 5, 5)

            if draw_paths and i > 0:
                # Update unpacking for previous waypoint to include linear_path
                prev_x, prev_y, prev_z, prev_rotation, prev_theta1, prev_theta2, _, _, prev_linear = waypoints[i - 1]
                prev_end_effector = QPoint((SIMULATION_WIDTH // 2)-SIMULATION_OFFSET + int(prev_x), WINDOW_HEIGHT // 2 - int(prev_y))
                self.draw_path_between_waypoints(painter, prev_end_effector, end_effector, 
                                               prev_theta1, prev_theta2, Wtheta1, Wtheta2, prev_linear)

    def draw_path_between_waypoints(self, painter, start_point, end_point, start_theta1, start_theta2, 
                                  end_theta1, end_theta2, is_linear_path):
        """Draw path between waypoints, either linear or curved."""
        # If it's a linear path, draw straight line
        painter.setRenderHint(QPainter.HighQualityAntialiasing)
        if is_linear_path:
            painter.setPen(QPen(RED, 1, Qt.SolidLine))
            painter.drawLine(start_point, end_point)
            return

        # If any angles are None (point outside workspace), draw dashed line
        if None in (start_theta1, start_theta2, end_theta1, end_theta2):
            painter.setPen(QPen(RED, 1, Qt.DashLine))
            painter.drawLine(start_point, end_point)
            return


        # Function to check if a point along the path is valid
        def is_valid_position(theta1, theta2):
            return -THETA1_LIMIT <= theta1 <= THETA1_LIMIT and -THETA2_LIMIT <= theta2 <= THETA2_LIMIT

        steps = 30  # Number of intermediate points to check
        points = []
        valid_path = True

        for i in range(steps + 1):
            t = i / steps
            # Smooth acceleration curve
            t = t * t * (3 - 2 * t)
            
            # Interpolate joint angles
            curr_theta1 = start_theta1 + (end_theta1 - start_theta1) * t
            curr_theta2 = start_theta2 + (end_theta2 - start_theta2) * t
            
            # Check if this position is valid
            if not is_valid_position(curr_theta1, curr_theta2):
                valid_path = False
                break
                
            # Calculate end effector position
            theta1_rad = math.radians(curr_theta1)
            theta2_rad = math.radians(curr_theta2)
            
            x = L1 * math.cos(theta1_rad) + L2 * math.cos(theta1_rad + theta2_rad)
            y = L1 * math.sin(theta1_rad) + L2 * math.sin(theta1_rad + theta2_rad)
            
            point = QPoint(
                int((SIMULATION_WIDTH // 2) - 100 + (x * SCALE)),
                int(WINDOW_HEIGHT // 2 + (y * SCALE))
            )
            points.append(point)

        # Draw the path
        if valid_path:
            painter.setPen(QPen(RED, 1, Qt.SolidLine))
            for i in range(len(points) - 1):
                painter.drawLine(points[i], points[i + 1])
        else:
            # Draw a dashed line to indicate invalid path
            painter.setPen(QPen(RED, 1, Qt.DashLine))
            painter.drawLine(start_point, end_point)

    def draw_z_axis(self, painter):
        z_axis_x = int(SIMULATION_WIDTH - 50)
        z_axis_y = 50
        z_axis_width = 20
        z_axis_height = WINDOW_HEIGHT - 100

        painter.setPen(QPen(WHITE, 1))
        painter.setBrush(QBrush(BLACK))
        painter.drawRect(z_axis_x, z_axis_y, z_axis_width, z_axis_height)

        z_position = int(z_axis_y + z_axis_height - ((self.z - Z_MIN) / (Z_MAX - Z_MIN)) * z_axis_height)
        painter.setBrush(QBrush(BLUE))
        painter.drawRect(z_axis_x, z_position, z_axis_width, 10)

    def draw_end_effector_rotation(self, painter, end_effector):
        # Calculate total rotation (theta1 + theta2 + theta3)
        absolute_angle = self.theta1 + self.theta2
        corrective_angle = self.rotation - absolute_angle
        
        # Convert to radians for drawing
        rotation_rad = math.radians(self.rotation)
        
        mask_length = 18
        arrow_1_x = int(end_effector.x() + 24 * math.cos(rotation_rad))
        arrow_1_y = int(end_effector.y() + 24 * math.sin(rotation_rad))
        arrow_2_x = int(end_effector.x() + 26 * math.cos(rotation_rad))
        arrow_2_y = int(end_effector.y() + 26 * math.sin(rotation_rad))
        arrow_3_x = int(end_effector.x() + 28 * math.cos(rotation_rad))
        arrow_3_y = int(end_effector.y() + 28 * math.sin(rotation_rad))

        mask_end_x = int(end_effector.x() + mask_length * math.cos(rotation_rad))
        mask_end_y = int(end_effector.y() + mask_length * math.sin(rotation_rad))

        # Draw the indicator
        painter.setPen(QPen(QColor(255, 255, 0), 4))
        painter.drawLine(QPoint(mask_end_x, mask_end_y), QPoint(arrow_1_x, arrow_1_y))
        painter.setPen(QPen(QColor(255, 255, 0), 3))
        painter.drawLine(QPoint(mask_end_x, mask_end_y), QPoint(arrow_2_x, arrow_2_y))
        painter.setPen(QPen(QColor(255, 255, 0), 2))
        painter.drawLine(QPoint(mask_end_x, mask_end_y), QPoint(arrow_3_x, arrow_3_y))
        
        return corrective_angle

    def calculate_robot_points(self, theta1, theta2):
        # Convert angles to radians
        theta1_rad = math.radians(theta1)
        theta2_rad = math.radians(theta2)

        # Calculate end-effector position
        x = L1 * math.cos(theta1_rad) + L2 * math.cos(theta1_rad + theta2_rad)
        y = L1 * math.sin(theta1_rad) + L2 * math.sin(theta1_rad + theta2_rad)

        # Calculate the points
        base = QPoint((SIMULATION_WIDTH // 2)-50, WINDOW_HEIGHT // 2)
        joint1 = QPoint(base.x() + int(L1 * SCALE * math.cos(theta1_rad)), base.y() + int(L1 * SCALE * math.sin(theta1_rad)))
        end_effector = QPoint(base.x() + int(x * SCALE), base.y() + int(y * SCALE))

        return base, joint1, end_effector

class SimulationWidget3D(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.theta1 = 0
        self.theta2 = 0
        self.z = 0
        self.rotation = 0
        self.setFixedSize(SIMULATION_WIDTH, WINDOW_HEIGHT)
        
        # Initialize PyBullet
        self.physics_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.setGravity(0, 0, 0)
        
        # Load URDF
        self.robot = p.loadURDF("scara_urdf/urdf/scara_urdf.urdf", [0, 0, 0], useFixedBase=1)
        
        # Configure camera view
        p.resetDebugVisualizerCamera(
            cameraDistance=0.6,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )
        
        # Start window embedding timer
        self.embed_timer = QTimer(self)
        self.embed_timer.timeout.connect(self.try_embed_window)
        self.embed_timer.start(100)  # Try every 100ms
        self.pybullet_hwnd = None
        
        # Update timer
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_robot)
        self.update_timer.start(16)  # 60 FPS

    def try_embed_window(self):
        """Try to find and embed the PyBullet window"""
        if self.pybullet_hwnd is None:
            # Find all windows from our process
            current_pid = win32process.GetCurrentProcessId()
            def callback(hwnd, results):
                if win32gui.IsWindowVisible(hwnd):
                    _, window_pid = win32process.GetWindowThreadProcessId(hwnd)
                    if window_pid == current_pid:
                        title = win32gui.GetWindowText(hwnd)
                        if "OpenGL" in title:  # PyBullet window typically has "OpenGL" in the title
                            results.append(hwnd)
                return True

            results = []
            win32gui.EnumWindows(callback, results)
            
            if results:
                self.pybullet_hwnd = results[0]
                parent_hwnd = int(self.winId())
                
                # Remove window decorations
                style = win32gui.GetWindowLong(self.pybullet_hwnd, win32con.GWL_STYLE)
                style = style & ~(win32con.WS_CAPTION | win32con.WS_THICKFRAME | win32con.WS_MINIMIZE | 
                                win32con.WS_MAXIMIZE | win32con.WS_SYSMENU)
                style = style | win32con.WS_CHILD
                win32gui.SetWindowLong(self.pybullet_hwnd, win32con.GWL_STYLE, style)
                
                # Remove extended window styles
                ex_style = win32gui.GetWindowLong(self.pybullet_hwnd, win32con.GWL_EXSTYLE)
                ex_style = ex_style & ~(win32con.WS_EX_CONTEXTHELP | win32con.WS_EX_WINDOWEDGE)
                win32gui.SetWindowLong(self.pybullet_hwnd, win32con.GWL_EXSTYLE, ex_style)
                
                # Set parent and position
                win32gui.SetParent(self.pybullet_hwnd, parent_hwnd)
                win32gui.MoveWindow(self.pybullet_hwnd, 0, 0, self.width(), self.height(), True)
                
                # Make sure the window is visible and properly sized
                win32gui.ShowWindow(self.pybullet_hwnd, win32con.SW_SHOW)
                
                # Stop the embed timer
                self.embed_timer.stop()
                print("Successfully embedded PyBullet window")

    def update_robot(self):
        """Update robot state"""
        try:
            # Convert angles to radians for PyBullet
            theta1_rad = math.radians(self.theta1)
            theta2_rad = math.radians(self.theta2)
            end_rotation = -(theta1_rad + theta2_rad + math.radians(self.rotation))
            
            # Update joint positions
            p.resetJointState(self.robot, 0, theta1_rad)
            p.resetJointState(self.robot, 1, theta2_rad)
            p.resetJointState(self.robot, 2, -self.z / 1000.0)
            p.resetJointState(self.robot, 3, end_rotation)
            
            # Step simulation
            p.stepSimulation()
            
        except Exception as e:
            print(f"Error updating PyBullet: {e}")

    def resizeEvent(self, event):
        """Handle resize events"""
        super().resizeEvent(event)
        if self.pybullet_hwnd:
            win32gui.MoveWindow(self.pybullet_hwnd, 0, 0, self.width(), self.height(), True)

    def showEvent(self, event):
        """Handle show events"""
        super().showEvent(event)
        if self.pybullet_hwnd:
            win32gui.ShowWindow(self.pybullet_hwnd, win32con.SW_SHOW)

    def hideEvent(self, event):
        """Handle hide events"""
        super().hideEvent(event)
        if self.pybullet_hwnd:
            win32gui.ShowWindow(self.pybullet_hwnd, win32con.SW_HIDE)

    def closeEvent(self, event):
        """Handle cleanup when closing"""
        try:
            self.update_timer.stop()
            self.embed_timer.stop()
            if self.pybullet_hwnd:
                # Restore window to normal state before closing
                style = win32gui.GetWindowLong(self.pybullet_hwnd, win32con.GWL_STYLE)
                style = style & ~win32con.WS_CHILD
                style = style | win32con.WS_POPUP
                win32gui.SetWindowLong(self.pybullet_hwnd, win32con.GWL_STYLE, style)
                win32gui.SetParent(self.pybullet_hwnd, None)
            p.disconnect(self.physics_client)
        except:
            pass
        super().closeEvent(event)

class Sidebar(QWidget):
    def __init__(self, simulation_widget):
        super().__init__()
        self.simulation_widget = simulation_widget
        self.waypoints_list = QListWidget()

        # Create the main layout
        layout = QVBoxLayout()

        self.initUI(layout)
        self.update_waypoints_list()
        self.initialize_file_handling()

    def initUI(self, layout):
        title_label = QLabel("Current Position")
        layout.addWidget(title_label)

        self.end_effector_x_label = QLabel("End Effector X:")
        self.end_effector_x_text = QLineEdit()
        self.end_effector_x_text.setReadOnly(True)

        self.end_effector_y_label = QLabel("End Effector Y:")
        self.end_effector_y_text = QLineEdit()
        self.end_effector_y_text.setReadOnly(True)

        self.end_effector_z_label = QLabel("End Effector Z:")
        self.end_effector_z_text = QLineEdit()
        self.end_effector_z_text.setReadOnly(True)

        self.end_effector_rotation_label = QLabel("End Effector Rotation:")
        self.end_effector_rotation_text = QLineEdit()
        self.end_effector_rotation_text.setReadOnly(True)

        self.theta1_label = QLabel("Theta1:")
        self.theta1_text = QLineEdit()
        self.theta1_text.setReadOnly(True)

        self.theta2_label = QLabel("Theta2:")
        self.theta2_text = QLineEdit()
        self.theta2_text.setReadOnly(True)
        
        self.theta3_label = QLabel("Theta3:")
        self.theta3_text = QLineEdit()
        self.theta3_text.setReadOnly(True)

        info_layout = QGridLayout()
        info_layout.addWidget(self.end_effector_x_label, 0, 0)
        info_layout.addWidget(self.end_effector_x_text, 0, 1)
        info_layout.addWidget(self.end_effector_y_label, 1, 0)
        info_layout.addWidget(self.end_effector_y_text, 1, 1)
        info_layout.addWidget(self.end_effector_z_label, 2, 0)
        info_layout.addWidget(self.end_effector_z_text, 2, 1)
        info_layout.addWidget(self.end_effector_rotation_label, 3, 0)
        info_layout.addWidget(self.end_effector_rotation_text, 3, 1)
        info_layout.addWidget(self.theta1_label, 4, 0)
        info_layout.addWidget(self.theta1_text, 4, 1)
        info_layout.addWidget(self.theta2_label, 5, 0)
        info_layout.addWidget(self.theta2_text, 5, 1)
        info_layout.addWidget(self.theta3_label, 6, 0)
        info_layout.addWidget(self.theta3_text, 6, 1)
        layout.addLayout(info_layout)

        title_label = QLabel("Waypoints")
        layout.addWidget(title_label)

        input_layout = QGridLayout()
        x_label = QLabel("X:")
        self.x_input = QLineEdit()
        y_label = QLabel("Y:")
        self.y_input = QLineEdit()
        z_label = QLabel("Z:")
        self.z_input = QLineEdit()
        rotation_label = QLabel("Rotation:")
        self.rotation_input = QLineEdit()
        input_layout.addWidget(x_label, 0, 0)
        input_layout.addWidget(self.x_input, 0, 1)
        input_layout.addWidget(y_label, 1, 0)
        input_layout.addWidget(self.y_input, 1, 1)
        input_layout.addWidget(z_label, 2, 0)
        input_layout.addWidget(self.z_input, 2, 1)
        input_layout.addWidget(rotation_label, 3, 0)
        input_layout.addWidget(self.rotation_input, 3, 1)

        self.stop_at_point_check = QCheckBox("Stop at Point")
        self.duration_label = QLabel("Duration (s):")
        self.duration_input = QLineEdit()
        self.duration_input.setEnabled(False)  # Disabled by default
        self.stop_at_point_check.stateChanged.connect(self.toggle_duration_input)
        
        input_layout.addWidget(self.stop_at_point_check, 4, 0)
        input_layout.addWidget(self.duration_label, 5, 0)
        input_layout.addWidget(self.duration_input, 5, 1)
        self.linear_path_check = QCheckBox("Linear Path to Next Point")
        input_layout.addWidget(self.linear_path_check, 6, 0)  # Add after duration input
        layout.addLayout(input_layout)
        add_button = QPushButton("Add Waypoint")
        add_button.clicked.connect(self.add_waypoint)
        layout.addWidget(add_button)

        self.waypoints_list = QListWidget()
        self.waypoints_list.itemClicked.connect(self.select_waypoint)
        layout.addWidget(self.waypoints_list)

        # Add buttons for loading and saving waypoints
        load_button = QPushButton("Load Waypoints")
        load_button.clicked.connect(self.load_waypoints_from_csv)
        save_button = QPushButton("Save Waypoints")
        save_button.clicked.connect(self.save_waypoints_to_csv)
        button_layout = QHBoxLayout()
        button_layout.addWidget(load_button)
        button_layout.addWidget(save_button)
        layout.addLayout(button_layout)

        edit_layout = QGridLayout()
        edit_x_label = QLabel("Edit X:")
        self.edit_x_input = QLineEdit()
        edit_y_label = QLabel("Edit Y:")
        self.edit_y_input = QLineEdit()
        edit_z_label = QLabel("Edit Z:")
        self.edit_z_input = QLineEdit()
        edit_rotation_label = QLabel("Edit Rotation:")
        self.edit_rotation_input = QLineEdit()
        edit_layout.addWidget(edit_x_label, 0, 0)
        edit_layout.addWidget(self.edit_x_input, 0, 1)
        edit_layout.addWidget(edit_y_label, 1, 0)
        edit_layout.addWidget(self.edit_y_input, 1, 1)
        edit_layout.addWidget(edit_z_label, 2, 0)
        edit_layout.addWidget(self.edit_z_input, 2, 1)
        edit_layout.addWidget(edit_rotation_label, 3, 0)
        edit_layout.addWidget(self.edit_rotation_input, 3, 1)
        self.edit_stop_at_point_check = QCheckBox("Stop at Point")
        self.edit_duration_label = QLabel("Duration (s):")
        self.edit_duration_input = QLineEdit()
        self.edit_duration_input.setEnabled(False)
        self.edit_stop_at_point_check.stateChanged.connect(self.toggle_edit_duration_input)
        
        edit_layout.addWidget(self.edit_stop_at_point_check, 4, 0)
        edit_layout.addWidget(self.edit_duration_label, 5, 0)
        edit_layout.addWidget(self.edit_duration_input, 5, 1)
        self.edit_linear_path_check = QCheckBox("Linear Path to Next Point")
        edit_layout.addWidget(self.edit_linear_path_check, 6, 0)  # Add after edit duration input
        layout.addLayout(edit_layout)

        update_button = QPushButton("Update")
        update_button.clicked.connect(self.update_waypoint)
        delete_button = QPushButton("Delete")
        delete_button.clicked.connect(self.delete_waypoint)
        button_layout = QHBoxLayout()
        button_layout.addWidget(update_button)
        button_layout.addWidget(delete_button)
        layout.addLayout(button_layout)

        # Animation controls
        animation_layout = QVBoxLayout()
        animation_buttons_layout = QHBoxLayout()
        self.play_button = QPushButton("Play")
        self.play_button.clicked.connect(self.play_animation)
        animation_buttons_layout.addWidget(self.play_button)

        self.loop_button = QPushButton("Loop")
        self.loop_button.setCheckable(True)
        animation_buttons_layout.addWidget(self.loop_button)

        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.pause_animation)
        animation_buttons_layout.addWidget(self.pause_button)

        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_animation)
        animation_buttons_layout.addWidget(self.stop_button)
        animation_layout.addLayout(animation_buttons_layout)

        layout.addLayout(animation_layout)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.animate)
        self.animation_speed = 10
        self.waypoint_index = 0
        self.is_animating = False
        self.is_paused = False
        self.reset_position = True
        self.loop_animation = False
        self.reverse_animation = False

    def validate_input_fields(self):
        """Validate all input fields before adding or updating waypoints"""
        error_messages = []
        
        # Validate X coordinate
        try:
            x = self.x_input.text().strip()
            if not x:
                error_messages.append("X coordinate is required")
            else:
                x = float(x)
                if abs(x) > (L1 + L2):
                    error_messages.append(f"X coordinate must be between {-(L1 + L2)} and {L1 + L2}")
        except ValueError:
            error_messages.append("X coordinate must be a number")

        # Validate Y coordinate
        try:
            y = self.y_input.text().strip()
            if not y:
                error_messages.append("Y coordinate is required")
            else:
                y = float(y)
                if abs(y) > (L1 + L2):
                    error_messages.append(f"Y coordinate must be between {-(L1 + L2)} and {L1 + L2}")
        except ValueError:
            error_messages.append("Y coordinate must be a number")

        # Validate Z coordinate
        try:
            z = self.z_input.text().strip()
            if not z:
                error_messages.append("Z coordinate is required")
            else:
                z = float(z)
                if not (Z_MIN <= z <= Z_MAX):
                    error_messages.append(f"Z coordinate must be between {Z_MIN} and {Z_MAX}")
        except ValueError:
            error_messages.append("Z coordinate must be a number")

        # Validate rotation
        try:
            rotation = self.rotation_input.text().strip()
            if not rotation:
                error_messages.append("Rotation is required")
            else:
                rotation = float(rotation)
                if not (-360 <= rotation <= 360):
                    error_messages.append("Rotation must be between -360 and 360 degrees")
        except ValueError:
            error_messages.append("Rotation must be a number")

        # Validate duration if stop at point is checked
        if self.stop_at_point_check.isChecked():
            try:
                duration = self.duration_input.text().strip()
                if not duration:
                    error_messages.append("Duration is required when 'Stop at Point' is checked")
                else:
                    duration = float(duration)
                    if duration <= 0:
                        error_messages.append("Duration must be greater than 0")
            except ValueError:
                error_messages.append("Duration must be a number")

        if error_messages:
            error_dialog = QMessageBox()
            error_dialog.setIcon(QMessageBox.Critical)
            error_dialog.setWindowTitle("Input Error")
            error_dialog.setText("\n".join(error_messages))
            error_dialog.exec_()
            return None

        # If validation passes, return the validated values
        return {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'rotation': float(rotation),
            'duration': float(duration) if self.stop_at_point_check.isChecked() else 0.0
        }

    def validate_edit_fields(self):
        """Validate all edit fields before updating waypoints"""
        error_messages = []
        
        # Validate X coordinate
        try:
            x = self.edit_x_input.text().strip()
            if not x:
                error_messages.append("X coordinate is required")
            else:
                x = float(x)
                if abs(x) > (L1 + L2):
                    error_messages.append(f"X coordinate must be between {-(L1 + L2)} and {L1 + L2}")
        except ValueError:
            error_messages.append("X coordinate must be a number")

        # Validate Y coordinate
        try:
            y = self.edit_y_input.text().strip()
            if not y:
                error_messages.append("Y coordinate is required")
            else:
                y = float(y)
                if abs(y) > (L1 + L2):
                    error_messages.append(f"Y coordinate must be between {-(L1 + L2)} and {L1 + L2}")
        except ValueError:
            error_messages.append("Y coordinate must be a number")

        # Validate Z coordinate
        try:
            z = self.edit_z_input.text().strip()
            if not z:
                error_messages.append("Z coordinate is required")
            else:
                z = float(z)
                if not (Z_MIN <= z <= Z_MAX):
                    error_messages.append(f"Z coordinate must be between {Z_MIN} and {Z_MAX}")
        except ValueError:
            error_messages.append("Z coordinate must be a number")

        # Validate rotation
        try:
            rotation = self.edit_rotation_input.text().strip()
            if not rotation:
                error_messages.append("Rotation is required")
            else:
                rotation = float(rotation)
                if not (-360 <= rotation <= 360):
                    error_messages.append("Rotation must be between -360 and 360 degrees")
        except ValueError:
            error_messages.append("Rotation must be a number")

        # Validate duration if stop at point is checked
        if self.edit_stop_at_point_check.isChecked():
            try:
                duration = self.edit_duration_input.text().strip()
                if not duration:
                    error_messages.append("Duration is required when 'Stop at Point' is checked")
                else:
                    duration = float(duration)
                    if duration <= 0:
                        error_messages.append("Duration must be greater than 0")
            except ValueError:
                error_messages.append("Duration must be a number")

        if error_messages:
            error_dialog = QMessageBox()
            error_dialog.setIcon(QMessageBox.Critical)
            error_dialog.setWindowTitle("Input Error")
            error_dialog.setText("\n".join(error_messages))
            error_dialog.exec_()
            return None

        # If validation passes, return the validated values
        return {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'rotation': float(rotation),
            'duration': float(duration) if self.edit_stop_at_point_check.isChecked() else 0.0
        }

    def toggle_duration_input(self, state):
        self.duration_input.setEnabled(state == Qt.Checked)
        if state != Qt.Checked:
            self.duration_input.clear()

    def toggle_edit_duration_input(self, state):
        self.edit_duration_input.setEnabled(state == Qt.Checked)
        if state != Qt.Checked:
            self.edit_duration_input.clear()

    def update_waypoints_list(self):
        self.waypoints_list.clear()
        for i, waypoint in enumerate(waypoints):
            x, y, z, rotation, theta1, theta2, stop_at_point, duration, linear_path = waypoint
            # Calculate theta3 for display
            theta3 = rotation - (theta1 + theta2) if theta1 is not None and theta2 is not None else None
            
            if theta1 is None or theta2 is None:
                item = QListWidgetItem(f"{i+1}. ({x}, {y}, {z}, {rotation}) - Outside workspace")
            else:
                stop_text = f" - Stop for {duration}s" if stop_at_point else " - Pass through"
                linear_text = " - Linear path" if linear_path else ""
                theta3_text = f", Theta3: {theta3:.2f}" if theta3 is not None else ""
                item = QListWidgetItem(
                    f"{i+1}. ({x}, {y}, {z}, {rotation}) - " + 
                    f"Theta1: {theta1:.2f}, Theta2: {theta2:.2f}{theta3_text}{stop_text}{linear_text}"
                )
            self.waypoints_list.addItem(item)

# INVERSE KINEMATICS

    def calculate_target_angles(self, waypoint, prev_theta1=None, prev_theta2=None):
        x, y, _, _ = waypoint
        # Flip the y-coordinate to match the robot's coordinate system
        y = -y
        
        r = math.sqrt(x ** 2 + y ** 2)
        if r > L1 + L2:
            print(f"Warning: Point ({x}, {y}) is outside maximum reach")
            return None, None

        c2 = (x ** 2 + y ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
        if abs(c2) > 1:
            return None, None

        s2 = math.sqrt(1 - c2 ** 2)
        theta2_1 = math.degrees(math.atan2(s2, c2))  # elbow-up
        theta2_2 = math.degrees(math.atan2(-s2, c2))  # elbow-down
        
        # Base angle to target point
        base = math.degrees(math.atan2(y, x))
        k1 = L1 + L2 * math.cos(math.radians(theta2_1))
        k2_1 = L2 * math.sin(math.radians(theta2_1))
        k2_2 = L2 * math.sin(math.radians(theta2_2))
        
        theta1_1 = base - math.degrees(math.atan2(k2_1, k1))  # for elbow-up
        theta1_2 = base - math.degrees(math.atan2(k2_2, k1))  # for elbow-down

        # Check solutions against joint limits
        sol1_valid = -THETA1_LIMIT <= theta1_1 <= THETA1_LIMIT and -THETA2_LIMIT <= theta2_1 <= THETA2_LIMIT
        sol2_valid = -THETA1_LIMIT <= theta1_2 <= THETA1_LIMIT and -THETA2_LIMIT <= theta2_2 <= THETA2_LIMIT

        if not sol1_valid and not sol2_valid:
            return None, None

        # If we have previous angles, strongly prefer maintaining configuration
        if prev_theta1 is not None and prev_theta2 is not None:
            # Determine if previous configuration was elbow-up or elbow-down
            prev_elbow_up = prev_theta2 > 0
            
            # First try to maintain the same configuration
            if prev_elbow_up and sol1_valid:
                return theta1_1, theta2_1
            elif not prev_elbow_up and sol2_valid:
                return theta1_2, theta2_2
                
            # If that's not possible, use the other valid solution
            if sol1_valid:
                return theta1_1, theta2_1
            elif sol2_valid:
                return theta1_2, theta2_2
        else:
            # No previous angles - use x-coordinate preference
            if x < 0:  # Left half
                if sol2_valid:
                    return theta1_2, theta2_2
                elif sol1_valid:
                    return theta1_1, theta2_1
            else:  # Right half
                if sol1_valid:
                    return theta1_1, theta2_1
                elif sol2_valid:
                    return theta1_2, theta2_2

        return None, None

    def add_waypoint(self):
        """Add a new waypoint with input validation"""
        # First validate all inputs
        validated_data = self.validate_input_fields()
        if validated_data is None:
            return
        
        x = validated_data['x']
        y = validated_data['y']
        z = validated_data['z']
        rotation = validated_data['rotation']
        duration = validated_data['duration']
        
        stop_at_point = self.stop_at_point_check.isChecked()
        linear_path = self.linear_path_check.isChecked()
        
        # Get previous angles for configuration preference
        prev_theta1 = None
        prev_theta2 = None
        if waypoints:
            _, _, _, _, prev_theta1, prev_theta2, _, _, _ = waypoints[-1]
        
        # Calculate target angles
        theta1, theta2 = self.calculate_target_angles((x, y, z, rotation), prev_theta1, prev_theta2)
        
        # Check if point is reachable
        if theta1 is None or theta2 is None:
            error_dialog = QMessageBox()
            error_dialog.setIcon(QMessageBox.Warning)
            error_dialog.setWindowTitle("Unreachable Point")
            error_dialog.setText("The specified point is outside the robot's reachable workspace.")
            error_dialog.exec_()
            return
        
        # Add the waypoint
        waypoints.append((x, y, z, rotation, theta1, theta2, stop_at_point, duration, linear_path))
        self.update_waypoints_list()
        
        # Clear inputs
        self.x_input.clear()
        self.y_input.clear()
        self.z_input.clear()
        self.rotation_input.clear()
        self.stop_at_point_check.setChecked(False)
        self.linear_path_check.setChecked(False)
        self.duration_input.clear()
        self.simulation_widget.update()

    def update_waypoint(self):
        """Update existing waypoint with input validation"""
        index = self.waypoints_list.currentRow()
        if index == -1:
            error_dialog = QMessageBox()
            error_dialog.setIcon(QMessageBox.Warning)
            error_dialog.setWindowTitle("No Selection")
            error_dialog.setText("Please select a waypoint to update.")
            error_dialog.exec_()
            return
            
        # Validate all inputs
        validated_data = self.validate_edit_fields()
        if validated_data is None:
            return
        
        x = validated_data['x']
        y = validated_data['y']
        z = validated_data['z']
        rotation = validated_data['rotation']
        duration = validated_data['duration']
        
        stop_at_point = self.edit_stop_at_point_check.isChecked()
        linear_path = self.edit_linear_path_check.isChecked()
        
        # Get previous angles for configuration preference
        prev_theta1 = None
        prev_theta2 = None
        if index > 0:
            _, _, _, _, prev_theta1, prev_theta2, _, _, _ = waypoints[index - 1]
        
        # Calculate target angles
        theta1, theta2 = self.calculate_target_angles((x, y, z, rotation), prev_theta1, prev_theta2)
        
        # Check if point is reachable
        if theta1 is None or theta2 is None:
            error_dialog = QMessageBox()
            error_dialog.setIcon(QMessageBox.Warning)
            error_dialog.setWindowTitle("Unreachable Point")
            error_dialog.setText("The specified point is outside the robot's reachable workspace.")
            error_dialog.exec_()
            return
        
        # Update the waypoint
        waypoints[index] = (x, y, z, rotation, theta1, theta2, stop_at_point, duration, linear_path)
        self.update_waypoints_list()
        
        # Clear inputs
        self.edit_x_input.clear()
        self.edit_y_input.clear()
        self.edit_z_input.clear()
        self.edit_rotation_input.clear()
        self.edit_stop_at_point_check.setChecked(False)
        self.edit_linear_path_check.setChecked(False)
        self.edit_duration_input.clear()
        self.simulation_widget.update()

    def select_waypoint(self, item):
        index = self.waypoints_list.row(item)
        x, y, z, rotation, theta1, theta2, stop_at_point, duration, linear_path = waypoints[index]
        self.edit_x_input.setText(str(x))
        self.edit_y_input.setText(str(y))
        self.edit_z_input.setText(str(z))
        self.edit_rotation_input.setText(str(rotation))
        self.edit_stop_at_point_check.setChecked(stop_at_point)
        self.edit_linear_path_check.setChecked(linear_path)
        if stop_at_point:
            self.edit_duration_input.setText(str(duration))

    def delete_waypoint(self):
        index = self.waypoints_list.currentRow()
        if index != -1:
            waypoints.pop(index)
            self.update_waypoints_list()
            self.edit_x_input.clear()
            self.edit_y_input.clear()
            self.edit_z_input.clear()
            self.edit_rotation_input.clear()

    def play_animation(self):
        if self.is_paused:
            self.simulation_widget.motion_controller.reset_timing()
        self.is_animating = True
        self.is_paused = False
        self.timer.start(20)  # 50 FPS

    def pause_animation(self):
        self.is_paused = True
        self.timer.stop()

    def stop_animation(self):
        self.is_animating = False
        self.is_paused = False
        self.timer.stop()
        self.waypoint_index = 0
        self.reverse_animation = False
        self.reset_position = True
        if hasattr(self, 'wait_time'):
            delattr(self, 'wait_time')
        if hasattr(self, 'wait_start_time'):
            delattr(self, 'wait_start_time')

    def generate_linear_path_points(self, start_point, end_point, num_points=20):
        """Generate evenly spaced points along a straight line."""
        x1, y1, z1, r1, _, _, _, _, _ = start_point
        x2, y2, z2, r2, _, _, _, _, _ = end_point
        
        points = []
        
        # Calculate distance and adjust number of points
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        num_points = max(200, int(distance / 1))  # One point every 10mm
        
        # Store start point configuration to maintain it
        prev_theta1, prev_theta2 = self.calculate_target_angles((x1, y1, z1, r1))
        
        for i in range(num_points):
            t = i / (num_points - 1)
            # Linear interpolation
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            z = z1 + (z2 - z1) * t
            r = r1 + (r2 - r1) * t
            
            # Calculate angles for this point
            theta1, theta2 = self.calculate_target_angles((x, y, z, r), prev_theta1, prev_theta2)
            if theta1 is None or theta2 is None:
                continue
            
            # Store angles for next iteration
            prev_theta1, prev_theta2 = theta1, theta2
            
            # All intermediate points are pass-through
            points.append((x, y, z, r, theta1, theta2, False, 0.0, False))
            
        return points

    def animate(self):
        if not self.is_animating or self.waypoint_index >= len(waypoints):
            if self.loop_button.isChecked():
                self.waypoint_index = 0
            else:
                self.stop_animation()
                return

        # Initialize or check interpolated points
        if not hasattr(self, 'interpolated_points'):
            self.interpolated_points = []
            self.interpolated_index = 0

        # Generate interpolated points if needed
        if not self.interpolated_points:
            current_point = waypoints[self.waypoint_index]
            
            if (self.waypoint_index < len(waypoints) - 1 and current_point[8]):  # linear_path flag
                next_point = waypoints[self.waypoint_index + 1]
                self.interpolated_points = self.generate_linear_path_points(current_point, next_point)
                if not self.interpolated_points:
                    self.interpolated_points = [current_point]
                self.interpolated_index = 0
            else:
                self.interpolated_points = [current_point]
                self.interpolated_index = 0

        # Get current target point
        current_point = self.interpolated_points[self.interpolated_index]
        x, y, z, rotation, theta1, theta2, stop_at_point, duration, _ = current_point

        if theta1 is None or theta2 is None:
            self.interpolated_index += 1
            if self.interpolated_index >= len(self.interpolated_points):
                self.interpolated_points = []
                self.waypoint_index += 1
            return

        # Set target for motion controller
        self.simulation_widget.motion_controller.set_target(theta1, theta2, z, rotation)

        # Update motion using existing acceleration control
        current_theta1, current_theta2, current_z, current_rotation, vel1, vel2 = \
            self.simulation_widget.motion_controller.update()

        # Update simulation widget positions
        self.simulation_widget.theta1 = current_theta1
        self.simulation_widget.theta2 = current_theta2
        self.simulation_widget.z = current_z
        self.simulation_widget.rotation = current_rotation

        # Calculate current end effector position
        theta1_rad = math.radians(current_theta1)
        theta2_rad = math.radians(current_theta2)
        current_x = L1 * math.cos(theta1_rad) + L2 * math.cos(theta1_rad + theta2_rad)
        current_y = -(L1 * math.sin(theta1_rad) + L2 * math.sin(theta1_rad + theta2_rad))

        # Calculate theta3 (corrective angle)
        absolute_angle = current_theta1 + current_theta2
        theta3 = current_rotation - absolute_angle

        # Check if we're at target and need to wait
        is_waiting = False
        if self.simulation_widget.motion_controller.is_at_target():
            if stop_at_point:
                current_time = time.time()
                if not hasattr(self, 'wait_start_time'):
                    self.wait_start_time = current_time
                    
                elapsed_wait = current_time - self.wait_start_time
                if elapsed_wait < duration:
                    is_waiting = True
                else:
                    delattr(self, 'wait_start_time')
                    self.interpolated_index += 1
                    if self.interpolated_index >= len(self.interpolated_points):
                        self.interpolated_points = []
                        self.waypoint_index += 1
            else:
                self.interpolated_index += 1
                if self.interpolated_index >= len(self.interpolated_points):
                    self.interpolated_points = []
                    self.waypoint_index += 1

        # Update displays
        self.update_end_effector_info(current_x, current_y, current_z, current_rotation,
                                    current_theta1, current_theta2)
        self.graph_widget.update_plot(current_theta1, current_theta2, theta3, vel1, vel2, is_waiting)

        # Trigger simulation update
        self.simulation_widget.update_simulation()
        QApplication.processEvents()

    def update_end_effector_info(self, end_effector_x, end_effector_y, end_effector_z, end_effector_rotation, theta1, theta2):
        absolute_angle = theta1 + theta2
        theta3 = end_effector_rotation - absolute_angle
        
        self.end_effector_x_text.setText(f"{end_effector_x:.2f}")
        self.end_effector_y_text.setText(f"{end_effector_y:.2f}")
        self.end_effector_z_text.setText(f"{end_effector_z:.2f}")
        self.end_effector_rotation_text.setText(f"{end_effector_rotation:.2f}")
        self.theta1_text.setText(f"{theta1:.2f}")
        self.theta2_text.setText(f"{theta2:.2f}")
        self.theta3_text.setText(f"{theta3:.2f}")

    def initialize_file_handling(self):
        """Initialize file handling system - call this in __init__"""
        # Create a dummy file operation to initialize the system
        try:
            with open('_init.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['init'])
            import os
            if os.path.exists('_init.csv'):
                os.remove('_init.csv')
        except Exception as e:
            print(f"Initialization note: {str(e)}")

    def save_waypoints_to_csv(self):
        try:
            if not waypoints:
                print("No waypoints to save")
                return

            file_path, _ = QFileDialog.getSaveFileName(
                self,
                "Save Waypoints",
                "",
                "CSV Files (*.csv)"
            )
            
            if not file_path:
                return

            if not file_path.lower().endswith('.csv'):
                file_path += '.csv'

            try:
                with open(file_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    # Write header with theta3
                    writer.writerow(['X', 'Y', 'Z', 'Rotation', 'Theta1', 'Theta2', 'Theta3', 
                                'StopAtPoint', 'Duration', 'LinearPath'])
                    # Write data
                    for wp in waypoints:
                        x, y, z, rotation, theta1, theta2, stop_at_point, duration, linear_path = wp
                        # Calculate theta3 for each waypoint
                        theta3 = rotation - (theta1 + theta2) if theta1 is not None and theta2 is not None else None
                        writer.writerow([
                            f"{x:.6f}",  # x
                            f"{y:.6f}",  # y 
                            f"{z:.6f}",  # z
                            f"{rotation:.6f}",  # rotation
                            'None' if theta1 is None else f"{theta1:.6f}",  # theta1
                            'None' if theta2 is None else f"{theta2:.6f}",  # theta2
                            'None' if theta3 is None else f"{theta3:.6f}",  # theta3
                            1 if stop_at_point else 0,  # stop_at_point
                            f"{duration:.6f}",  # duration
                            1 if linear_path else 0   # linear_path
                        ])
                print(f"Successfully saved {len(waypoints)} waypoints to {file_path}")
                        
            except csv.Error as e:
                print(f"CSV Error: {e}")
                return
                        
        except Exception as e:
            print(f"Save error: {e}")

    def load_waypoints_from_csv(self):
        try:
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                "Load Waypoints",
                "",
                "CSV Files (*.csv)"
            )
            
            if not file_path:
                return

            loaded_points = []
            try:
                with open(file_path, 'r', newline='') as f:
                    reader = csv.reader(f)
                    header = next(reader)  # Read header
                    
                    # Verify header format
                    expected_header = ['X', 'Y', 'Z', 'Rotation', 'Theta1', 'Theta2', 'Theta3', 
                                    'StopAtPoint', 'Duration', 'LinearPath']
                    if not all(h1.lower() == h2.lower() for h1, h2 in zip(header[:len(expected_header)], expected_header)):
                        print("Warning: CSV header format doesn't match expected format")
                    
                    for i, row in enumerate(reader, start=2):  # start=2 because row 1 is header
                        try:
                            x = float(row[0].strip())
                            y = float(row[1].strip())
                            z = float(row[2].strip())
                            rotation = float(row[3].strip())
                            theta1 = float(row[4].strip()) if row[4].strip().lower() != 'none' else None
                            theta2 = float(row[5].strip()) if row[5].strip().lower() != 'none' else None
                            # Theta3 is read but not stored in waypoints as it's calculated
                            stop_at_point = bool(int(row[7].strip()))
                            duration = float(row[8].strip())
                            linear_path = bool(int(row[9].strip())) if len(row) > 9 else False
                            
                            # Store waypoint without theta3 as it's dynamically calculated
                            loaded_points.append((x, y, z, rotation, theta1, theta2, 
                                            stop_at_point, duration, linear_path))
                        except (ValueError, IndexError) as e:
                            print(f"Warning: Error parsing row {i}: {e}")
                            continue

                if loaded_points:
                    global waypoints
                    waypoints = loaded_points
                    self.update_waypoints_list()
                    self.simulation_widget.update()
                    print(f"Successfully loaded {len(loaded_points)} waypoints")
                else:
                    print("Warning: No valid waypoints found in CSV")
                    
            except csv.Error as e:
                print(f"CSV Error: {e}")
                return
                
        except Exception as e:
            print(f"Load error: {e}")

if __name__ == '__main__':
    multiprocessing.freeze_support()
    app = QApplication(sys.argv)

    # Create widgets
    graph_widget = AngleGraphWidget()
    simulation_widget = TabbedSimulationWidget()  # Use the new tabbed widget
    sidebar = Sidebar(simulation_widget)

    # Store graph widget reference in sidebar for updates
    sidebar.graph_widget = graph_widget

    # Create layout
    layout = QHBoxLayout()
    layout.addWidget(graph_widget)
    layout.addWidget(simulation_widget)
    layout.addWidget(sidebar)

    main_window = QWidget()
    main_window.setLayout(layout)
    main_window.show()

    # Initial position update
    end_effector_x = L1 * math.cos(math.radians(45)) + L2 * math.cos(math.radians(45 + 45))
    end_effector_y = -1 * (L1 * math.sin(math.radians(45)) + L2 * math.sin(math.radians(45 + 45)))
    end_effector_z = 0
    end_effector_rotation = 0
    sidebar.update_end_effector_info(end_effector_x, end_effector_y, end_effector_z, end_effector_rotation, 45, 45)

    sys.exit(app.exec_())