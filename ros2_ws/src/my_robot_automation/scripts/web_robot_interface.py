#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, render_template_string, jsonify, request
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import JointState
import threading
import json

# Web interface HTML template
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control Interface</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .control-panel { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin: 20px 0; }
        .control-btn { padding: 15px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; transition: all 0.3s; }
        .move-btn { background: #4CAF50; color: white; }
        .turn-btn { background: #2196F3; color: white; }
        .gripper-btn { background: #FF9800; color: white; }
        .emergency-btn { background: #f44336; color: white; }
        .control-btn:hover { transform: translateY(-2px); box-shadow: 0 4px 8px rgba(0,0,0,0.2); }
        .status-panel { background: #f5f5f5; padding: 15px; border-radius: 5px; margin: 20px 0; }
        .status-item { display: flex; justify-content: space-between; margin: 5px 0; }
        .log-panel { background: #000; color: #0f0; padding: 15px; border-radius: 5px; height: 200px; overflow-y: scroll; font-family: monospace; }
        .speed-control { margin: 10px 0; }
        .speed-slider { width: 100%; margin: 10px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Robot Control Interface</h1>
        
        <div class="status-panel">
            <h3>🤖 Real Robot Status</h3>
            <div class="status-item">
                <span>Position:</span>
                <span id="position">x: 0, y: 0, θ: 0</span>
            </div>
            <div class="status-item">
                <span>Lifter:</span>
                <span id="lifter">0.0 cm</span>
            </div>
            <div class="status-item">
                <span>Servo 1:</span>
                <span id="servo1">0°</span>
            </div>
            <div class="status-item">
                <span>Servo 2:</span>
                <span id="servo2">0°</span>
            </div>
            <div class="status-item">
                <span>Front Ultrasonic:</span>
                <span id="ultrasonic_front">-- cm</span>
            </div>
            <div class="status-item">
                <span>Front IR:</span>
                <span id="ir_front">-- cm</span>
            </div>
            <div class="status-item">
                <span>Line Sensor:</span>
                <span id="line_sensor">--</span>
            </div>
        </div>
        
        <div class="speed-control">
            <label>Speed: <span id="speed-value">0.5</span></label>
            <input type="range" class="speed-slider" id="speed-slider" min="0.1" max="1.0" step="0.1" value="0.5">
        </div>
        
        <div class="control-panel">
            <!-- Omni Wheel Movement -->
            <button class="move-btn control-btn" onclick="moveRobot('forward')">⬆️ Forward</button>
            <button class="move-btn control-btn" onclick="moveRobot('backward')">⬇️ Backward</button>
            <button class="move-btn control-btn" onclick="stopRobot()">⏹️ Stop</button>
            
            <button class="turn-btn control-btn" onclick="turnRobot('left')">↩️ Turn Left</button>
            <button class="turn-btn control-btn" onclick="turnRobot('right')">↪️ Turn Right</button>
            <button class="turn-btn control-btn" onclick="moveRobot('strafe_left')">⬅️ Strafe Left</button>
            
            <!-- Lifter Control -->
            <button class="gripper-btn control-btn" onclick="controlLifter('up')">⬆️ Lifter Up</button>
            <button class="gripper-btn control-btn" onclick="controlLifter('down')">⬇️ Lifter Down</button>
            
            <!-- Servo Control -->
            <button class="emergency-btn control-btn" onclick="controlServos('home')">🏠 Servo Home</button>
        </div>
        
        <div class="control-panel">
            <h3>🔧 Servo Control</h3>
            <div style="display: grid; grid-template-columns: repeat(5, 1fr); gap: 10px;">
                <div>
                    <label>Servo 1:</label><br>
                    <input type="range" min="0" max="180" value="90" id="servo1-slider" onchange="setServo(1, this.value)">
                </div>
                <div>
                    <label>Servo 2:</label><br>
                    <input type="range" min="0" max="180" value="90" id="servo2-slider" onchange="setServo(2, this.value)">
                </div>
                <div>
                    <label>Servo 3:</label><br>
                    <input type="range" min="0" max="180" value="90" id="servo3-slider" onchange="setServo(3, this.value)">
                </div>
                <div>
                    <label>Servo 4:</label><br>
                    <input type="range" min="0" max="180" value="90" id="servo4-slider" onchange="setServo(4, this.value)">
                </div>
                <div>
                    <label>Servo 5:</label><br>
                    <input type="range" min="0" max="180" value="90" id="servo5-slider" onchange="setServo(5, this.value)">
                </div>
            </div>
        </div>
        
        <div class="status-panel">
            <h3>System Log</h3>
            <div class="log-panel" id="log-panel"></div>
        </div>
    </div>

    <script>
        let currentSpeed = 0.5;
        
        // Speed control
        document.getElementById('speed-slider').oninput = function() {
            currentSpeed = parseFloat(this.value);
            document.getElementById('speed-value').textContent = currentSpeed.toFixed(1);
        };
        
        // Robot control functions
        async function moveRobot(direction) {
            const response = await fetch('/api/robot/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({direction: direction, speed: currentSpeed})
            });
            const result = await response.json();
            addLog(`Move ${direction}: ${result.message}`);
        }
        
        async function turnRobot(direction) {
            const response = await fetch('/api/robot/turn', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({direction: direction, speed: currentSpeed})
            });
            const result = await response.json();
            addLog(`Turn ${direction}: ${result.message}`);
        }
        
        async function stopRobot() {
            const response = await fetch('/api/robot/stop', {method: 'POST'});
            const result = await response.json();
            addLog(`Stop: ${result.message}`);
        }
        
        async function controlLifter(action) {
            const response = await fetch('/api/robot/lifter', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: action, speed: currentSpeed})
            });
            const result = await response.json();
            addLog(`Lifter ${action}: ${result.message}`);
        }
        
        async function setServo(servoNum, angle) {
            const response = await fetch('/api/robot/servo', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({servo: servoNum, angle: parseFloat(angle)})
            });
            const result = await response.json();
            addLog(`Servo ${servoNum} to ${angle}°: ${result.message}`);
        }
        
        async function controlServos(action) {
            const response = await fetch('/api/robot/servos', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: action})
            });
            const result = await response.json();
            addLog(`Servos ${action}: ${result.message}`);
        }
        
        async function emergencyStop() {
            const response = await fetch('/api/robot/emergency', {method: 'POST'});
            const result = await response.json();
            addLog(`EMERGENCY STOP: ${result.message}`);
        }
        
        // Status updates
        async function updateStatus() {
            try {
                const response = await fetch('/api/robot/status');
                const status = await response.json();
                
                document.getElementById('position').textContent = `x: ${status.x.toFixed(1)}, y: ${status.y.toFixed(1)}, θ: ${status.theta.toFixed(1)}`;
                document.getElementById('lifter').textContent = `${status.lifter.toFixed(1)} cm`;
                document.getElementById('servo1').textContent = `${status.servo1.toFixed(0)}°`;
                document.getElementById('servo2').textContent = `${status.servo2.toFixed(0)}°`;
                document.getElementById('ultrasonic_front').textContent = `${status.ultrasonic_front.toFixed(1)} cm`;
                document.getElementById('ir_front').textContent = `${status.ir_front.toFixed(1)} cm`;
                document.getElementById('line_sensor').textContent = `0x${status.line_sensor.toString(16).toUpperCase()}`;
            } catch (error) {
                console.error('Status update failed:', error);
            }
        }
        
        function addLog(message) {
            const logPanel = document.getElementById('log-panel');
            const timestamp = new Date().toLocaleTimeString();
            logPanel.innerHTML += `[${timestamp}] ${message}<br>`;
            logPanel.scrollTop = logPanel.scrollHeight;
        }
        
        // Update status every second
        setInterval(updateStatus, 1000);
        
        // Initial status update
        updateStatus();
        addLog('Robot Control Interface initialized');
    </script>
</body>
</html>
"""

class WebRobotInterface(Node):
    def __init__(self):
        super().__init__('web_robot_interface')
        
        # ROS2 publishers for real hardware
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Omni wheels
        self.lifter_pub = self.create_publisher(Float32, '/lifter/position', 10)  # Lifter
        self.servo_pub = self.create_publisher(JointState, '/servo/command', 10)  # Servos
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.lifter_position = 0.0
        self.servo_positions = [90.0, 90.0, 90.0, 90.0, 90.0]  # 5 servos at 90 degrees
        self.ultrasonic_front = 2.0
        self.ir_front = 0.3
        self.line_sensor = 0xAA
        
        # Flask web server
        self.app = Flask(__name__)
        self.setup_routes()
        
        self.get_logger().info('Web Robot Interface started')
        self.get_logger().info('Access the interface at: http://localhost:5000')
    
    def setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)
        
        @self.app.route('/api/robot/move', methods=['POST'])
        def move_robot():
            data = request.get_json()
            direction = data.get('direction', 'stop')
            speed = data.get('speed', 0.5)
            
            cmd = Twist()
            if direction == 'forward':
                cmd.linear.x = speed
            elif direction == 'backward':
                cmd.linear.x = -speed
            elif direction == 'strafe_left':
                cmd.linear.y = speed
            elif direction == 'strafe_right':
                cmd.linear.y = -speed
            
            self.cmd_vel_pub.publish(cmd)
            return jsonify({'success': True, 'message': f'Moving {direction} at {speed} m/s'})
        
        @self.app.route('/api/robot/turn', methods=['POST'])
        def turn_robot():
            data = request.get_json()
            direction = data.get('direction', 'left')
            speed = data.get('speed', 0.5)
            
            cmd = Twist()
            if direction == 'left':
                cmd.angular.z = speed
            elif direction == 'right':
                cmd.angular.z = -speed
            
            self.cmd_vel_pub.publish(cmd)
            return jsonify({'success': True, 'message': f'Turning {direction} at {speed} rad/s'})
        
        @self.app.route('/api/robot/stop', methods=['POST'])
        def stop_robot():
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return jsonify({'success': True, 'message': 'Robot stopped'})
        
        @self.app.route('/api/robot/lifter', methods=['POST'])
        def control_lifter():
            data = request.get_json()
            action = data.get('action', 'up')
            speed = data.get('speed', 0.5)
            
            if action == 'up':
                self.lifter_position += speed * 0.1  # Move up
            elif action == 'down':
                self.lifter_position -= speed * 0.1  # Move down
            
            # Clamp position
            self.lifter_position = max(0.0, min(10.0, self.lifter_position))
            
            cmd = Float32()
            cmd.data = self.lifter_position
            self.lifter_pub.publish(cmd)
            
            return jsonify({'success': True, 'message': f'Lifter moved {action} to {self.lifter_position:.1f} cm'})
        
        @self.app.route('/api/robot/servo', methods=['POST'])
        def set_servo():
            data = request.get_json()
            servo_num = data.get('servo', 1)
            angle = data.get('angle', 90.0)
            
            # Clamp angle
            angle = max(0.0, min(180.0, angle))
            
            # Update servo position
            if 1 <= servo_num <= 5:
                self.servo_positions[servo_num - 1] = angle
            
            # Publish servo command
            cmd = JointState()
            cmd.name = [f'servo_{i}' for i in range(1, 6)]
            cmd.position = self.servo_positions
            self.servo_pub.publish(cmd)
            
            return jsonify({'success': True, 'message': f'Servo {servo_num} set to {angle:.0f}°'})
        
        @self.app.route('/api/robot/servos', methods=['POST'])
        def control_servos():
            data = request.get_json()
            action = data.get('action', 'home')
            
            if action == 'home':
                self.servo_positions = [90.0, 90.0, 90.0, 90.0, 90.0]
            
            # Publish servo command
            cmd = JointState()
            cmd.name = [f'servo_{i}' for i in range(1, 6)]
            cmd.position = self.servo_positions
            self.servo_pub.publish(cmd)
            
            return jsonify({'success': True, 'message': f'All servos moved to {action} position'})
        
        @self.app.route('/api/robot/emergency', methods=['POST'])
        def emergency_stop():
            # Stop all movement
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
            # Move lifter to safe position
            lifter_cmd = Float32()
            lifter_cmd.data = 0.0  # Lower lifter
            self.lifter_pub.publish(lifter_cmd)
            
            # Move servos to safe position
            servo_cmd = JointState()
            servo_cmd.name = [f'servo_{i}' for i in range(1, 6)]
            servo_cmd.position = [90.0, 90.0, 90.0, 90.0, 90.0]  # Safe position
            self.servo_pub.publish(servo_cmd)
            
            return jsonify({'success': True, 'message': 'EMERGENCY STOP ACTIVATED - All actuators moved to safe position'})
        
        @self.app.route('/api/robot/status')
        def get_robot_status():
            return jsonify({
                'x': self.robot_x,
                'y': self.robot_y,
                'theta': self.robot_theta,
                'lifter': self.lifter_position,
                'servo1': self.servo_positions[0],
                'servo2': self.servo_positions[1],
                'ultrasonic_front': self.ultrasonic_front,
                'ir_front': self.ir_front,
                'line_sensor': self.line_sensor
            })
    
    def run_web_server(self):
        """Run Flask web server in a separate thread"""
        self.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main(args=None):
    rclpy.init(args=args)
    interface = WebRobotInterface()
    
    # Start web server in a separate thread
    web_thread = threading.Thread(target=interface.run_web_server)
    web_thread.daemon = True
    web_thread.start()
    
    try:
        rclpy.spin(interface)
    except KeyboardInterrupt:
        pass
    finally:
        interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
