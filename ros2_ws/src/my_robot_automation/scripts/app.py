"""
Main Flask Application
Web interface for Autonomous Mobile Manipulator control
"""

import time
import threading
import logging
from flask import Flask, request, jsonify, render_template_string
from config import FLASK_HOST, FLASK_PORT, FLASK_DEBUG, DEFAULT_SIMULATION_MODE
from mega_interface import MegaInterface
from sensor_manager import SensorManager
from gpio_controller import GPIOController

# ROS2 interface is optional (only available when ROS2 is installed)
try:
    from ros2_interface import ROS2Interface
    ROS2_AVAILABLE = True
except ImportError:
    ROS2Interface = None
    ROS2_AVAILABLE = False

logger = logging.getLogger(__name__)

# HTML template for the web interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Autonomous Mobile Manipulator Control</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { background: #2c3e50; color: white; padding: 20px; border-radius: 8px; margin-bottom: 20px; }
        .tabs { display: flex; margin-bottom: 20px; }
        .tab { padding: 10px 20px; background: #ecf0f1; border: none; cursor: pointer; margin-right: 5px; border-radius: 4px; }
        .tab.active { background: #3498db; color: white; }
        .tab-content { background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .hidden { display: none; }
        .control-group { margin-bottom: 20px; }
        .control-group label { display: block; margin-bottom: 5px; font-weight: bold; }
        .control-group input, .control-group select, .control-group button {
            padding: 8px; margin-right: 10px; border: 1px solid #ddd; border-radius: 4px;
        }
        .control-group button { background: #3498db; color: white; cursor: pointer; }
        .control-group button:hover { background: #2980b9; }
        .status { padding: 10px; margin: 10px 0; border-radius: 4px; }
        .status.success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
        .status.error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
        .status.info { background: #cce7ff; color: #004085; border: 1px solid #b3d7ff; }
        .log-container { background: #f8f9fa; border: 1px solid #dee2e6; border-radius: 4px; padding: 10px; max-height: 300px; overflow-y: auto; font-family: monospace; font-size: 12px; }
        .sensor-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; }
        .sensor-card { background: #f8f9fa; padding: 15px; border-radius: 8px; border: 1px solid #dee2e6; }
        .sensor-card h4 { margin-top: 0; color: #495057; }
        .sensor-value { font-size: 24px; font-weight: bold; color: #007bff; }
        .button-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 10px; }
        .btn { padding: 10px; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; }
        .btn-success { background: #28a745; color: white; }
        .btn-danger { background: #dc3545; color: white; }
        .btn-warning { background: #ffc107; color: black; }
        .btn-info { background: #17a2b8; color: white; }
        .btn-secondary { background: #6c757d; color: white; }
        .input-group { display: flex; align-items: center; margin-bottom: 10px; }
        .input-group label { min-width: 100px; margin-right: 10px; }
        .input-group input { flex: 1; }
        .description { font-size: 12px; color: #6c757d; margin-top: 5px; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 Autonomous Mobile Manipulator Control</h1>
            <p>Real-time control interface for robot navigation, manipulation, and sensor monitoring</p>
        </div>

        <div class="tabs">
            <button class="tab active" onclick="showTab('dashboard')">Dashboard</button>
            <button class="tab" onclick="showTab('movement')">Movement</button>
            <button class="tab" onclick="showTab('manipulation')">Manipulation</button>
            <button class="tab" onclick="showTab('sensors')">Sensors</button>
            <button class="tab" onclick="showTab('serial')">Direct Serial</button>
        </div>

        <!-- Dashboard Tab -->
        <div id="dashboard" class="tab-content">
            <h2>System Status</h2>
            <div id="system-status" class="status info">
                <strong>Status:</strong> <span id="status-text">Initializing...</span><br>
                <strong>Mega Connected:</strong> <span id="mega-status">Checking...</span><br>
                <strong>ROS2 Services:</strong> <span id="ros2-status">Checking...</span>
            </div>

            <h3>Quick Actions</h3>
            <div class="button-grid">
                <button class="btn btn-success" onclick="emergencyStop()">🚨 Emergency Stop</button>
                <button class="btn btn-warning" onclick="homeServos()">🏠 Home Servos</button>
                <button class="btn btn-info" onclick="testLimitSwitches()">🔍 Test Limit Switches</button>
            </div>
        </div>

        <!-- Movement Tab -->
        <div id="movement" class="tab-content hidden">
            <h2>Robot Movement Control</h2>

            <div class="control-group">
                <label for="speed-slider">Speed Control (0-100%):</label>
                <input type="range" id="speed-slider" min="0" max="100" value="50" oninput="updateSpeedDisplay()">
                <span id="speed-display">50%</span>
                <button class="btn btn-info" onclick="setSpeed()">Set Speed</button>
            </div>

            <div class="control-group">
                <label>Turbo Mode:</label>
                <button class="btn btn-warning" onclick="toggleTurbo()">🚀 Toggle Turbo</button>
            </div>

            <h3>Basic Movement</h3>
            <div class="button-grid">
                <button class="btn btn-primary" onclick="moveRobot('forward')">⬆️ Forward</button>
                <button class="btn btn-primary" onclick="moveRobot('backward')">⬇️ Backward</button>
                <button class="btn btn-primary" onclick="turnRobot('left')">⬅️ Turn Left</button>
                <button class="btn btn-primary" onclick="turnRobot('right')">➡️ Turn Right</button>
                <button class="btn btn-danger" onclick="stopRobot()">⏹️ Stop</button>
            </div>

            <h3>Individual Wheel Control</h3>
            <div class="control-group">
                <label for="wheel-select">Wheel:</label>
                <select id="wheel-select">
                    <option value="0">Front Left (0)</option>
                    <option value="1">Front Right (1)</option>
                    <option value="2">Rear Left (2)</option>
                    <option value="3">Rear Right (3)</option>
                </select>
            </div>
            <div class="control-group">
                <label for="wheel-speed">Speed (-100 to 100):</label>
                <input type="range" id="wheel-speed" min="-100" max="100" value="0" oninput="updateWheelSpeedDisplay()">
                <span id="wheel-speed-display">0</span>
                <button class="btn btn-info" onclick="controlWheel()">Set Wheel Speed</button>
                <button class="btn btn-danger" onclick="stopAllWheels()">Stop All Wheels</button>
            </div>

            <h3>Movement Presets</h3>
            <div class="button-grid">
                <button class="btn btn-secondary" onclick="executePreset('square')">🔳 Square Path</button>
                <button class="btn btn-secondary" onclick="executePreset('figure8')">∞ Figure-8</button>
                <button class="btn btn-secondary" onclick="executePreset('lawnmower')">🌾 Lawnmower</button>
                <button class="btn btn-secondary" onclick="executePreset('scan')">🔍 Scan Area</button>
            </div>
        </div>

        <!-- Manipulation Tab -->
        <div id="manipulation" class="tab-content hidden">
            <h2>Robot Manipulation Control</h2>

            <h3>Gripper Control</h3>
            <div class="button-grid">
                <button class="btn btn-success" onclick="controlGripper('open_full')">📂 Open (Full)</button>
                <button class="btn btn-info" onclick="controlGripper('open_half')">📂 Open (Half)</button>
                <button class="btn btn-danger" onclick="controlGripper('close')">📕 Close</button>
            </div>

            <div class="control-group">
                <label for="gripper-tilt">Gripper Tilt Angle (0-180°):</label>
                <input type="range" id="gripper-tilt" min="0" max="180" value="90" oninput="updateGripperTiltDisplay()">
                <span id="gripper-tilt-display">90°</span>
                <button class="btn btn-info" onclick="setGripperTilt()">Set Tilt</button>
            </div>

            <h3>Container Control</h3>
            <div class="control-group">
                <label for="container-id">Container ID:</label>
                <select id="container-id">
                    <option value="1">Container 1</option>
                    <option value="2">Container 2</option>
                    <option value="3">Container 3</option>
                </select>
            </div>
            <div class="button-grid">
                <button class="btn btn-success" onclick="controlContainer('open')">📦 Open</button>
                <button class="btn btn-danger" onclick="controlContainer('close')">📪 Close</button>
                <button class="btn btn-info" onclick="controlContainer('lock')">🔒 Lock</button>
                <button class="btn btn-warning" onclick="controlContainer('unlock')">🔓 Unlock</button>
            </div>
        </div>

        <!-- Sensors Tab -->
        <div id="sensors" class="tab-content hidden">
            <h2>Sensor Monitoring</h2>
            <button class="btn btn-info" onclick="refreshSensors()">🔄 Refresh Sensors</button>

            <h3>IR Distance Sensors</h3>
            <div class="sensor-grid" id="ir-sensors">
                <!-- IR sensors will be populated here -->
            </div>

            <h3>Ultrasonic Sensors</h3>
            <div class="sensor-grid" id="ultrasonic-sensors">
                <!-- Ultrasonic sensors will be populated here -->
            </div>

            <h3>IMU Data</h3>
            <div class="sensor-grid" id="imu-data">
                <!-- IMU data will be populated here -->
            </div>

            <h3>Sensor Health</h3>
            <div id="sensor-health" class="status info">
                <!-- Sensor health status will be populated here -->
            </div>
        </div>

        <!-- Direct Serial Tab -->
        <div id="serial" class="tab-content hidden">
            <h2>Direct Serial Communication</h2>

            <div class="control-group">
                <label for="serial-command">Command:</label>
                <input type="text" id="serial-command" placeholder="Enter command (e.g., f, s, p)">
                <button class="btn btn-success" onclick="sendSerialCommand()">📤 Send Command</button>
            </div>

            <h3>Quick Commands</h3>
            <div class="button-grid">
                <button class="btn btn-info" onclick="sendQuickCommand('p')">📊 Status (p)</button>
                <button class="btn btn-warning" onclick="sendQuickCommand('ls')">🔍 Limit Switch Test (ls)</button>
                <button class="btn btn-success" onclick="sendQuickCommand('se')">🛡️ Enable Safety (se)</button>
                <button class="btn btn-danger" onclick="sendQuickCommand('sd')">⚠️ Disable Safety (sd)</button>
                <button class="btn btn-secondary" onclick="sendQuickCommand('wstop')">⏹️ Stop Wheels (wstop)</button>
            </div>

            <h3>Gripper Commands</h3>
            <div class="button-grid">
                <button class="btn btn-success" onclick="sendQuickCommand('no')">📂 Open Full (no)</button>
                <button class="btn btn-info" onclick="sendQuickCommand('nh')">📂 Open Half (nh)</button>
                <button class="btn btn-danger" onclick="sendQuickCommand('nc')">📕 Close (nc)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('ta90')">📐 Tilt 90° (ta90)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('ga45')">🔧 Gripper 45° (ga45)</button>
            </div>

            <h3>Movement Commands</h3>
            <div class="button-grid">
                <button class="btn btn-primary" onclick="sendQuickCommand('f')">⬆️ Forward (f)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('b')">⬇️ Backward (b)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('l')">⬅️ Left (l)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('r')">➡️ Right (r)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('q')">↺ Turn Left (q)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('e')">↻ Turn Right (e)</button>
                <button class="btn btn-danger" onclick="sendQuickCommand('s')">⏹️ Stop (s)</button>
            </div>

            <h3>Command Log</h3>
            <div class="log-container" id="serial-log">
                <!-- Serial command log will be populated here -->
            </div>
        </div>
    </div>

    <script>
        let currentTab = 'dashboard';

        function showTab(tabName) {
            document.querySelectorAll('.tab-content').forEach(tab => tab.classList.add('hidden'));
            document.querySelectorAll('.tab').forEach(tab => tab.classList.remove('active'));
            document.getElementById(tabName).classList.remove('hidden');
            event.target.classList.add('active');
            currentTab = tabName;
        }

        function updateSpeedDisplay() {
            const slider = document.getElementById('speed-slider');
            const display = document.getElementById('speed-display');
            display.textContent = slider.value + '%';
        }

        function updateWheelSpeedDisplay() {
            const slider = document.getElementById('wheel-speed');
            const display = document.getElementById('wheel-speed-display');
            display.textContent = slider.value;
        }

        function updateGripperTiltDisplay() {
            const slider = document.getElementById('gripper-tilt');
            const display = document.getElementById('gripper-tilt-display');
            display.textContent = slider.value + '°';
        }

        // API Functions
        async function apiCall(endpoint, data = {}) {
            try {
                const response = await fetch(endpoint, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(data)
                });
                const result = await response.json();
                showStatus(result.success ? 'success' : 'error', result.message || result.error);
                return result;
            } catch (error) {
                showStatus('error', 'Network error: ' + error.message);
                return { success: false, error: error.message };
            }
        }

        function showStatus(type, message) {
            const statusDiv = document.createElement('div');
            statusDiv.className = `status ${type}`;
            statusDiv.textContent = message;
            document.querySelector('.container').insertBefore(statusDiv, document.querySelector('.tabs'));
            setTimeout(() => statusDiv.remove(), 5000);
        }

        // Movement controls
        async function setSpeed() {
            const speed = document.getElementById('speed-slider').value;
            await apiCall('/api/robot/speed', { speed: parseInt(speed) });
        }

        async function toggleTurbo() {
            await apiCall('/api/robot/turbo');
        }

        async function moveRobot(direction) {
            await apiCall('/api/robot/move', { direction, speed: 0.5, duration: 1.0 });
        }

        async function turnRobot(direction) {
            await apiCall('/api/robot/turn', { direction, speed: 0.5 });
        }

        async function stopRobot() {
            await apiCall('/api/robot/stop');
        }

        async function controlWheel() {
            const wheelId = document.getElementById('wheel-select').value;
            const speed = document.getElementById('wheel-speed').value;
            await apiCall('/api/robot/wheels/' + wheelId, { speed: parseInt(speed) });
        }

        async function stopAllWheels() {
            await apiCall('/api/robot/wheels/stop');
        }

        async function executePreset(preset) {
            showStatus('info', `Executing ${preset} preset...`);
            // Implementation would depend on preset logic
        }

        // Manipulation controls
        async function controlGripper(command) {
            await apiCall('/api/robot/picker/gripper', { command });
        }

        async function setGripperTilt() {
            const angle = document.getElementById('gripper-tilt').value;
            await apiCall('/api/robot/picker/gripper_tilt', { angle: parseInt(angle) });
        }

        async function controlContainer(action) {
            const containerId = document.getElementById('container-id').value;
            await apiCall('/api/robot/containers/' + containerId, { action });
        }

        // Emergency controls
        async function emergencyStop() {
            await apiCall('/api/robot/emergency-stop');
        }

        async function homeServos() {
            await apiCall('/api/robot/servos/home');
        }

        async function testLimitSwitches() {
            await apiCall('/api/serial/send', { command: 'ls' });
        }

        // Serial communication
        async function sendSerialCommand() {
            const command = document.getElementById('serial-command').value.trim();
            if (!command) return;

            const result = await apiCall('/api/serial/send', { command });
            if (result.success) {
                addToSerialLog(`> ${command}`, 'command');
                addToSerialLog(`< ${result.response || 'OK'}`, 'response');
                document.getElementById('serial-command').value = '';
            }
        }

        async function sendQuickCommand(command) {
            await apiCall('/api/serial/send', { command });
            addToSerialLog(`> ${command}`, 'command');
        }

        function addToSerialLog(text, type) {
            const logContainer = document.getElementById('serial-log');
            const logEntry = document.createElement('div');
            logEntry.className = type;
            logEntry.textContent = `[${new Date().toLocaleTimeString()}] ${text}`;
            logContainer.appendChild(logEntry);
            logContainer.scrollTop = logContainer.scrollHeight;
        }

        // Sensor monitoring
        async function refreshSensors() {
            try {
                const response = await fetch('/api/robot/sensors');
                const data = await response.json();

                if (data.success) {
                    updateSensorDisplay(data.data);
                } else {
                    showStatus('error', 'Failed to refresh sensors');
                }
            } catch (error) {
                showStatus('error', 'Network error refreshing sensors');
            }
        }

        function updateSensorDisplay(sensorData) {
            // Update IR sensors
            const irContainer = document.getElementById('ir-sensors');
            irContainer.innerHTML = '';
            ['left_front', 'left_back', 'right_front', 'right_back', 'back_left', 'back_right'].forEach(sensor => {
                const value = sensorData.laser_sensors?.[sensor] || 'N/A';
                const card = document.createElement('div');
                card.className = 'sensor-card';
                card.innerHTML = `
                    <h4>IR ${sensor.replace('_', ' ').toUpperCase()}</h4>
                    <div class="sensor-value">${value} mm</div>
                `;
                irContainer.appendChild(card);
            });

            // Update ultrasonic sensors
            const usContainer = document.getElementById('ultrasonic-sensors');
            usContainer.innerHTML = '';
            ['front_left', 'front_right'].forEach(sensor => {
                const value = sensorData.ultrasonic_sensors?.[sensor] || 'N/A';
                const card = document.createElement('div');
                card.className = 'sensor-card';
                card.innerHTML = `
                    <h4>Ultrasonic ${sensor.replace('_', ' ').toUpperCase()}</h4>
                    <div class="sensor-value">${value} mm</div>
                `;
                usContainer.appendChild(card);
            });

            // Update IMU data
            const imuContainer = document.getElementById('imu-data');
            imuContainer.innerHTML = '';
            if (sensorData.imu) {
                const imu = sensorData.imu;
                ['orientation', 'angular_velocity', 'linear_acceleration'].forEach(type => {
                    if (imu[type]) {
                        const card = document.createElement('div');
                        card.className = 'sensor-card';
                        card.innerHTML = `
                            <h4>IMU ${type.replace('_', ' ').toUpperCase()}</h4>
                            <div>X: ${imu[type].x?.toFixed(2) || 'N/A'}</div>
                            <div>Y: ${imu[type].y?.toFixed(2) || 'N/A'}</div>
                            <div>Z: ${imu[type].z?.toFixed(2) || 'N/A'}</div>
                        `;
                        imuContainer.appendChild(card);
                    }
                });
            }
        }

        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            updateSpeedDisplay();
            updateWheelSpeedDisplay();
            updateGripperTiltDisplay();

            // Auto-refresh sensors every 5 seconds
            setInterval(refreshSensors, 5000);

            // Keyboard shortcuts for serial commands
            document.getElementById('serial-command').addEventListener('keypress', function(e) {
                if (e.key === 'Enter') {
                    sendSerialCommand();
                }
            });
        });
    </script>
</body>
</html>
"""

class FlaskApp:
    """Main Flask application for robot control"""

    def __init__(self, mega_interface=None, sensor_manager=None, ros2_interface=None, simulation_mode=False):
        self.app = Flask(__name__)
        self.mega = mega_interface
        self.sensors = sensor_manager
        self.ros2 = ros2_interface
        self.simulation_mode = simulation_mode

        # Setup routes
        self._setup_routes()

        logger.info("Flask application initialized")

    def _setup_routes(self):
        """Setup Flask routes"""

        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)

        @self.app.route('/health')
        def health():
            return jsonify({
                'status': 'healthy',
                'timestamp': time.time(),
                'simulation_mode': self.simulation_mode
            })

        # Status and diagnostics
        @self.app.route('/api/status')
        def get_status():
            return self._get_status()

        @self.app.route('/api/robot/sensors')
        def get_sensors():
            return self._get_sensors()

        @self.app.route('/api/robot/sensors/diagnostics')
        def get_sensor_diagnostics():
            return self._get_sensor_diagnostics()

        # Movement control
        @self.app.route('/api/robot/speed', methods=['POST'])
        def set_speed():
            return self._set_speed()

        @self.app.route('/api/robot/turbo', methods=['POST'])
        def toggle_turbo():
            return self._toggle_turbo()

        @self.app.route('/api/robot/move', methods=['POST'])
        def move_robot():
            return self._move_robot()

        @self.app.route('/api/robot/turn', methods=['POST'])
        def turn_robot():
            return self._turn_robot()

        @self.app.route('/api/robot/stop', methods=['POST'])
        def stop_robot():
            return self._stop_robot()

        # Wheel control
        @self.app.route('/api/robot/wheels/<int:wheel_id>', methods=['POST'])
        def control_wheel(wheel_id):
            return self._control_wheel(wheel_id)

        @self.app.route('/api/robot/wheels/stop', methods=['POST'])
        def stop_all_wheels():
            return self._stop_all_wheels()

        # Manipulation
        @self.app.route('/api/robot/picker/gripper', methods=['POST'])
        def control_gripper():
            return self._control_gripper()

        @self.app.route('/api/robot/picker/gripper_tilt', methods=['POST'])
        def set_gripper_tilt():
            return self._set_gripper_tilt()

        # Serial communication
        @self.app.route('/api/serial/send', methods=['POST'])
        def send_serial_command():
            return self._send_serial_command()

        # Emergency controls
        @self.app.route('/api/robot/emergency-stop', methods=['POST'])
        def emergency_stop():
            return self._emergency_stop()

    def _get_status(self):
        """Get system status"""
        mega_connected = self.mega.mega_connected if self.mega else False
        ros2_services = self.ros2.actuator_clients_created if self.ros2 else False

        return jsonify({
            'success': True,
            'data': {
                'simulation_mode': self.simulation_mode,
                'mega_connected': mega_connected,
                'ros2_services_available': ros2_services,
                'actuators_available': True,  # GPIO always available
                'timestamp': time.time()
            }
        })

    def _get_sensors(self):
        """Get sensor data"""
        if self.sensors:
            sensor_data = self.sensors.read_all_sensors()
            return jsonify({
                'success': True,
                'data': sensor_data,
                'timestamp': time.time()
            })
        else:
            return jsonify({
                'success': False,
                'error': 'Sensor manager not available',
                'timestamp': time.time()
            }), 500

    def _get_sensor_diagnostics(self):
        """Get sensor diagnostics"""
        if self.sensors:
            health = self.sensors.get_sensor_health()
            return jsonify({
                'success': True,
                'data': {
                    'simulation_mode': self.simulation_mode,
                    'sensor_health': health,
                    'adc_config': {
                        'vref': 3.3,
                        'resolution': 1024,
                        'spi_speed': 1350000
                    }
                },
                'timestamp': time.time()
            })
        else:
            return jsonify({
                'success': False,
                'error': 'Sensor manager not available',
                'timestamp': time.time()
            }), 500

    def _set_speed(self):
        """Set robot speed"""
        try:
            data = request.get_json()
            if not data or 'speed' not in data:
                return jsonify({'success': False, 'error': 'Speed value required', 'timestamp': time.time()}), 400

            speed_percent = int(data['speed'])
            if not (0 <= speed_percent <= 100):
                return jsonify({'success': False, 'error': 'Speed must be 0-100', 'timestamp': time.time()}), 400

            if self.mega:
                success = self.mega.set_speed(speed_percent)
                if success:
                    return jsonify({
                        'success': True,
                        'message': f'Speed set to {speed_percent}%',
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({'success': False, 'error': 'Failed to set speed', 'timestamp': time.time()}), 500
            else:
                return jsonify({'success': False, 'error': 'Mega interface not available', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Speed control error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _toggle_turbo(self):
        """Toggle turbo mode"""
        try:
            if self.mega:
                success = self.mega.toggle_turbo()
                if success:
                    return jsonify({
                        'success': True,
                        'message': 'Turbo mode toggled',
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({'success': False, 'error': 'Failed to toggle turbo', 'timestamp': time.time()}), 500
            else:
                return jsonify({'success': False, 'error': 'Mega interface not available', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Turbo toggle error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _move_robot(self):
        """Move robot in specified direction"""
        try:
            data = request.get_json()
            direction = data.get('direction')
            speed = data.get('speed', 0.5)
            duration = data.get('duration', 0.0)

            if not direction:
                return jsonify({'success': False, 'error': 'Direction required', 'timestamp': time.time()}), 400

            # Try Mega first, then ROS2, then GPIO
            success = False
            if self.mega:
                success = self.mega.move_robot(direction, speed)
            if not success and self.ros2:
                success = self.ros2.call_move_robot(direction, speed, duration)

            if success:
                return jsonify({
                    'success': True,
                    'message': f'Robot moved {direction}',
                    'timestamp': time.time()
                })
            else:
                return jsonify({'success': False, 'error': 'Failed to move robot', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Move robot error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _turn_robot(self):
        """Turn robot in specified direction"""
        try:
            data = request.get_json()
            direction = data.get('direction')
            speed = data.get('speed', 0.5)

            if not direction:
                return jsonify({'success': False, 'error': 'Direction required', 'timestamp': time.time()}), 400

            # Try Mega first, then ROS2
            success = False
            if self.mega:
                success = self.mega.turn_robot(direction, speed)
            if not success and self.ros2:
                # ROS2 turn would use move_robot with turn commands
                pass

            if success:
                return jsonify({
                    'success': True,
                    'message': f'Robot turned {direction}',
                    'timestamp': time.time()
                })
            else:
                return jsonify({'success': False, 'error': 'Failed to turn robot', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Turn robot error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _stop_robot(self):
        """Stop robot movement"""
        try:
            # Try Mega first, then ROS2
            success = False
            if self.mega:
                success = self.mega.stop_robot()
            if not success and self.ros2:
                # ROS2 stop would use move_robot with stop command
                pass

            if success:
                return jsonify({
                    'success': True,
                    'message': 'Robot stopped',
                    'timestamp': time.time()
                })
            else:
                return jsonify({'success': False, 'error': 'Failed to stop robot', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Stop robot error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _control_wheel(self, wheel_id):
        """Control individual wheel"""
        try:
            data = request.get_json()
            speed = data.get('speed', 0.0)

            if self.mega:
                success = self.mega.control_wheel(wheel_id, speed)
                if success:
                    return jsonify({
                        'success': True,
                        'message': f'Wheel {wheel_id} set to speed {speed}',
                        'wheel': wheel_id,
                        'speed': speed,
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({'success': False, 'error': f'Failed to control wheel {wheel_id}', 'timestamp': time.time()}), 500
            else:
                return jsonify({'success': False, 'error': 'Mega interface not available', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Wheel control error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _stop_all_wheels(self):
        """Stop all wheels"""
        try:
            if self.mega:
                success = self.mega.stop_all_wheels()
                if success:
                    return jsonify({
                        'success': True,
                        'action': 'stop_all_wheels',
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({'success': False, 'error': 'Failed to stop wheels', 'timestamp': time.time()}), 500
            else:
                return jsonify({'success': False, 'error': 'Mega interface not available', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Wheel stop error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _control_gripper(self):
        """Control gripper"""
        try:
            data = request.get_json()
            command = data.get('command')

            if not command:
                return jsonify({'success': False, 'error': 'Command required', 'timestamp': time.time()}), 400

            # Try Mega first, then ROS2
            success = False
            if self.mega:
                success = self.mega.control_gripper(command)
            if not success and self.ros2:
                success = self.ros2.call_control_gripper(command)

            if success:
                return jsonify({
                    'success': True,
                    'message': f'Gripper {command} executed',
                    'command': command,
                    'timestamp': time.time()
                })
            else:
                return jsonify({'success': False, 'error': f'Failed to execute gripper command {command}', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Gripper control error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _set_gripper_tilt(self):
        """Set gripper tilt angle"""
        try:
            data = request.get_json()
            angle = data.get('angle', 90.0)

            if not (0 <= angle <= 180):
                return jsonify({'success': False, 'error': 'Angle must be 0-180', 'timestamp': time.time()}), 400

            # Try Mega first, then ROS2
            success = False
            if self.mega:
                success = self.mega.set_gripper_tilt(angle)
            if not success and self.ros2:
                success = self.ros2.call_set_gripper_tilt(angle)

            if success:
                return jsonify({
                    'success': True,
                    'message': f'Gripper tilt set to {angle}°',
                    'angle': angle,
                    'timestamp': time.time()
                })
            else:
                return jsonify({'success': False, 'error': f'Failed to set gripper tilt to {angle}°', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Gripper tilt error: {str(e)}')
            return jsonify({'success': False, 'error': str(e), 'timestamp': time.time()}), 500

    def _send_serial_command(self):
        """Send direct serial command to Mega"""
        try:
            data = request.get_json()
            if not data or 'command' not in data:
                return jsonify({'success': False, 'error': 'Command required', 'timestamp': time.time()}), 400

            command = data['command'].strip().lower()
            if len(command) < 1:
                return jsonify({'success': False, 'error': 'Command cannot be empty', 'timestamp': time.time()}), 400

            # Validate command
            valid_commands = [
                'f', 'b', 'l', 'r', 'q', 'e', 'z', 'x', 'c', 'w', 't', 'y', 'a', 'j', 's', 'p', 'v', 'o',
                '5', '6', '7', '8', '9', '0', 'u', 'd', '1', '2', '3', '4', 'g', 'h',
                'mu', 'md', 'mc', 'no', 'nc', 'nh', 'sr', 'ls', 'se', 'sd'
            ]

            # Check for angle commands
            is_valid = command in valid_commands
            if not is_valid:
                if command.startswith('ta') and len(command) >= 3:
                    try:
                        angle = int(command[2:])
                        is_valid = (0 <= angle <= 180)
                    except ValueError:
                        is_valid = False
                elif command.startswith('ga') and len(command) >= 3:
                    try:
                        angle = int(command[2:])
                        is_valid = (0 <= angle <= 180)
                    except ValueError:
                        is_valid = False
                elif command.startswith('w') and len(command) >= 2:
                    # Wheel commands: w[0-3][-100-100]
                    try:
                        parts = command.split('w')
                        if len(parts) == 2 and parts[0] == '':
                            wheel_part, speed_part = parts[1].split('[') if '[' in parts[1] else (parts[1], '')
                            if not speed_part:
                                # w[0-3] format
                                wheel_id = int(wheel_part)
                                speed = 50  # default
                            else:
                                # w[0-3][speed] format
                                wheel_id = int(wheel_part)
                                speed = int(speed_part.rstrip(']'))
                            is_valid = (0 <= wheel_id <= 3) and (-100 <= speed <= 100)
                        elif parts[1] == 'stop':
                            is_valid = True  # wstop command
                    except (ValueError, IndexError):
                        is_valid = False

            if not is_valid:
                return jsonify({'success': False, 'error': f'Invalid command. Valid: {", ".join(valid_commands)}, ta<angle>, ga<angle>, w[0-3]<speed>, wstop', 'timestamp': time.time()}), 400

            if self.mega:
                success = self.mega.send_command_to_mega(command)
                if success:
                    return jsonify({
                        'success': True,
                        'message': f'Command sent: {command}',
                        'command': command,
                        'mega_connected': self.mega.mega_connected,
                        'timestamp': time.time()
                    })
                else:
                    return jsonify({'success': False, 'error': f'Failed to send command: {command}', 'timestamp': time.time()}), 500
            else:
                return jsonify({'success': False, 'error': 'Mega interface not available', 'timestamp': time.time()}), 500

        except Exception as e:
            logger.error(f'Direct serial command error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _emergency_stop(self):
        """Emergency stop all systems"""
        try:
            # Stop all movement
            if self.mega:
                self.mega.stop_all_wheels()
                self.mega.stop_robot()

            return jsonify({
                'success': True,
                'message': 'Emergency stop executed',
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Emergency stop error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def run(self, host=FLASK_HOST, port=FLASK_PORT, debug=FLASK_DEBUG):
        """Run the Flask application"""
        logger.info(f"Starting Flask web server on http://{host}:{port}")
        self.app.run(host=host, port=port, debug=debug)
