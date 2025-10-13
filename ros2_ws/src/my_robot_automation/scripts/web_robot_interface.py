#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, render_template_string, jsonify, request
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
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
            <h3>Robot Status</h3>
            <div class="status-item">
                <span>Battery:</span>
                <span id="battery">--%</span>
            </div>
            <div class="status-item">
                <span>Obstacle:</span>
                <span id="obstacle">None</span>
            </div>
            <div class="status-item">
                <span>Position:</span>
                <span id="position">(0, 0)</span>
            </div>
        </div>
        
        <div class="speed-control">
            <label>Speed: <span id="speed-value">0.5</span></label>
            <input type="range" class="speed-slider" id="speed-slider" min="0.1" max="1.0" step="0.1" value="0.5">
        </div>
        
        <div class="control-panel">
            <button class="move-btn control-btn" onclick="moveRobot('forward')">⬆️ Forward</button>
            <button class="move-btn control-btn" onclick="moveRobot('backward')">⬇️ Backward</button>
            <button class="move-btn control-btn" onclick="stopRobot()">⏹️ Stop</button>
            
            <button class="turn-btn control-btn" onclick="turnRobot('left')">↩️ Turn Left</button>
            <button class="turn-btn control-btn" onclick="turnRobot('right')">↪️ Turn Right</button>
            <button class="turn-btn control-btn" onclick="moveRobot('strafe_left')">⬅️ Strafe Left</button>
            
            <button class="gripper-btn control-btn" onclick="controlGripper('open')">🤏 Open Gripper</button>
            <button class="gripper-btn control-btn" onclick="controlGripper('close')">✊ Close Gripper</button>
            <button class="emergency-btn control-btn" onclick="emergencyStop()">🛑 EMERGENCY STOP</button>
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
        
        async function controlGripper(action) {
            const response = await fetch('/api/robot/gripper', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: action})
            });
            const result = await response.json();
            addLog(`Gripper ${action}: ${result.message}`);
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
                
                document.getElementById('battery').textContent = status.battery + '%';
                document.getElementById('obstacle').textContent = status.obstacle;
                document.getElementById('position').textContent = `(${status.x.toFixed(1)}, ${status.y.toFixed(1)})`;
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
        
        # ROS2 publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper_control', 10)
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.battery_level = 100.0
        self.obstacle_detected = False
        
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
        
        @self.app.route('/api/robot/gripper', methods=['POST'])
        def control_gripper():
            data = request.get_json()
            action = data.get('action', 'open')
            
            cmd = Bool()
            cmd.data = (action == 'open')
            self.gripper_pub.publish(cmd)
            
            return jsonify({'success': True, 'message': f'Gripper {action}ed'})
        
        @self.app.route('/api/robot/emergency', methods=['POST'])
        def emergency_stop():
            # Stop all movement
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
            # Close gripper for safety
            gripper_cmd = Bool()
            gripper_cmd.data = False
            self.gripper_pub.publish(gripper_cmd)
            
            return jsonify({'success': True, 'message': 'EMERGENCY STOP ACTIVATED'})
        
        @self.app.route('/api/robot/status')
        def get_robot_status():
            return jsonify({
                'battery': self.battery_level,
                'obstacle': 'Detected' if self.obstacle_detected else 'None',
                'x': self.robot_x,
                'y': self.robot_y
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
