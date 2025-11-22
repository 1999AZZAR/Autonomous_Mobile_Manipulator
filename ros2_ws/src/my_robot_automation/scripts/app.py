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
        :root {
            --primary-bg: #0f172a;
            --secondary-bg: #1e293b;
            --accent-bg: #334155;
            --card-bg: #1e293b;
            --border-color: #334155;
            --text-primary: #f8fafc;
            --text-secondary: #94a3b8;
            --text-muted: #64748b;
            --accent-blue: #3b82f6;
            --accent-green: #10b981;
            --accent-purple: #8b5cf6;
            --accent-red: #ef4444;
            --accent-orange: #f59e0b;
            --gradient-primary: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            --gradient-success: linear-gradient(135deg, #10b981 0%, #059669 100%);
            --gradient-danger: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
            --gradient-warning: linear-gradient(135deg, #f59e0b 0%, #d97706 100%);
            --glass-bg: rgba(30, 41, 59, 0.8);
            --glass-border: rgba(51, 65, 85, 0.3);
            --shadow-soft: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
            --shadow-medium: 0 10px 15px -3px rgba(0, 0, 0, 0.1);
            --shadow-large: 0 20px 25px -5px rgba(0, 0, 0, 0.1);
        }

        * {
            box-sizing: border-box;
        }

        body {
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            margin: 0;
            padding: 0;
            background: var(--primary-bg);
            color: var(--text-primary);
            line-height: 1.6;
            min-height: 100vh;
        }

        .container {
            max-width: 1400px;
            margin: 0 auto;
            padding: 20px;
        }

        .header {
            background: var(--gradient-primary);
            color: white;
            padding: 30px;
            border-radius: 16px;
            margin-bottom: 30px;
            box-shadow: var(--shadow-large);
            backdrop-filter: blur(10px);
            border: 1px solid var(--glass-border);
        }

        .header h1 {
            margin: 0 0 10px 0;
            font-size: 2.5rem;
            font-weight: 700;
            background: linear-gradient(135deg, #ffffff 0%, #e2e8f0 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
        }

        .header p {
            margin: 0;
            opacity: 0.9;
            font-size: 1.1rem;
        }

        .tabs {
            display: flex;
            margin-bottom: 30px;
            background: var(--secondary-bg);
            border-radius: 12px;
            padding: 6px;
            box-shadow: var(--shadow-medium);
            overflow-x: auto;
        }

        .tab {
            padding: 12px 24px;
            background: transparent;
            border: none;
            cursor: pointer;
            border-radius: 8px;
            color: var(--text-secondary);
            font-weight: 500;
            font-size: 14px;
            transition: all 0.3s ease;
            white-space: nowrap;
            min-width: fit-content;
        }

        .tab:hover {
            background: var(--accent-bg);
            color: var(--text-primary);
            transform: translateY(-1px);
        }

        .tab.active {
            background: var(--gradient-primary);
            color: white;
            box-shadow: var(--shadow-soft);
            font-weight: 600;
        }

        .tab-content {
            background: var(--glass-bg);
            backdrop-filter: blur(20px);
            padding: 30px;
            border-radius: 16px;
            box-shadow: var(--shadow-large);
            border: 1px solid var(--glass-border);
            animation: fadeIn 0.3s ease;
        }

        @keyframes fadeIn {
            from { opacity: 0; transform: translateY(10px); }
            to { opacity: 1; transform: translateY(0); }
        }

        .hidden { display: none; }

        .control-group {
            margin-bottom: 25px;
            background: var(--card-bg);
            padding: 20px;
            border-radius: 12px;
            border: 1px solid var(--border-color);
        }

        .control-group h3 {
            margin: 0 0 15px 0;
            color: var(--text-primary);
            font-size: 1.25rem;
            font-weight: 600;
        }

        .control-group label {
            display: block;
            margin-bottom: 8px;
            font-weight: 500;
            color: var(--text-primary);
            font-size: 14px;
        }

        .control-group input,
        .control-group select,
        .control-group textarea {
            padding: 12px 16px;
            margin-right: 12px;
            border: 2px solid var(--border-color);
            border-radius: 8px;
            background: var(--secondary-bg);
            color: var(--text-primary);
            font-size: 14px;
            transition: all 0.3s ease;
        }

        .control-group input:focus,
        .control-group select:focus,
        .control-group textarea:focus {
            outline: none;
            border-color: var(--accent-blue);
            box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.1);
        }

        .control-group button {
            background: var(--gradient-primary);
            color: white;
            cursor: pointer;
            border: none;
            border-radius: 8px;
            padding: 12px 20px;
            font-weight: 500;
            transition: all 0.3s ease;
            box-shadow: var(--shadow-soft);
        }

        .control-group button:hover {
            transform: translateY(-2px);
            box-shadow: var(--shadow-medium);
        }

        .control-group button:active {
            transform: translateY(0);
        }

        .status {
            padding: 16px 20px;
            margin: 15px 0;
            border-radius: 12px;
            border-left: 4px solid;
            backdrop-filter: blur(10px);
        }

        .status.success {
            background: rgba(16, 185, 129, 0.1);
            color: #10b981;
            border-left-color: #10b981;
        }

        .status.error {
            background: rgba(239, 68, 68, 0.1);
            color: #ef4444;
            border-left-color: #ef4444;
        }

        .status.info {
            background: rgba(59, 130, 246, 0.1);
            color: #3b82f6;
            border-left-color: #3b82f6;
        }

        .status.warning {
            background: rgba(245, 158, 11, 0.1);
            color: #f59e0b;
            border-left-color: #f59e0b;
        }

        .log-container {
            background: var(--secondary-bg);
            border: 1px solid var(--border-color);
            border-radius: 8px;
            padding: 16px;
            max-height: 400px;
            overflow-y: auto;
            font-family: 'JetBrains Mono', 'Fira Code', monospace;
            font-size: 13px;
            line-height: 1.4;
        }

        .sensor-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 20px;
        }

        .sensor-card {
            background: var(--glass-bg);
            backdrop-filter: blur(10px);
            padding: 20px;
            border-radius: 12px;
            border: 1px solid var(--glass-border);
            box-shadow: var(--shadow-soft);
            transition: all 0.3s ease;
        }

        .sensor-card:hover {
            transform: translateY(-2px);
            box-shadow: var(--shadow-medium);
        }

        .sensor-card h4 {
            margin: 0 0 15px 0;
            color: var(--text-primary);
            font-size: 1.1rem;
            font-weight: 600;
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .sensor-value {
            font-size: 2rem;
            font-weight: 700;
            color: var(--accent-blue);
            margin: 10px 0;
            display: flex;
            align-items: baseline;
            gap: 4px;
        }

        .sensor-unit {
            font-size: 0.8rem;
            color: var(--text-secondary);
            font-weight: 400;
        }

        .button-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
            gap: 12px;
        }

        .btn {
            padding: 14px 20px;
            border: none;
            border-radius: 10px;
            cursor: pointer;
            font-size: 14px;
            font-weight: 500;
            transition: all 0.3s ease;
            box-shadow: var(--shadow-soft);
            position: relative;
            overflow: hidden;
        }

        .btn::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.2), transparent);
            transition: left 0.5s;
        }

        .btn:hover::before {
            left: 100%;
        }

        .btn:hover {
            transform: translateY(-2px);
            box-shadow: var(--shadow-medium);
        }

        .btn:active {
            transform: translateY(0);
        }

        .btn-success {
            background: var(--gradient-success);
            color: white;
        }

        .btn-danger {
            background: var(--gradient-danger);
            color: white;
        }

        .btn-warning {
            background: var(--gradient-warning);
            color: black;
        }

        .btn-info {
            background: linear-gradient(135deg, #06b6d4 0%, #0891b2 100%);
            color: white;
        }

        .btn-secondary {
            background: linear-gradient(135deg, #64748b 0%, #475569 100%);
            color: white;
        }

        .btn-primary {
            background: var(--gradient-primary);
            color: white;
        }

        .input-group {
            display: flex;
            align-items: center;
            margin-bottom: 12px;
        }

        .input-group label {
            min-width: 120px;
            margin-right: 15px;
            font-weight: 500;
        }

        .input-group input {
            flex: 1;
        }

        .description {
            font-size: 13px;
            color: var(--text-muted);
            margin-top: 6px;
            font-style: italic;
        }

        /* Slider styling */
        input[type="range"] {
            -webkit-appearance: none;
            appearance: none;
            height: 8px;
            background: var(--border-color);
            border-radius: 4px;
            outline: none;
        }

        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: var(--gradient-primary);
            cursor: pointer;
            box-shadow: var(--shadow-soft);
        }

        input[type="range"]::-moz-range-thumb {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: var(--gradient-primary);
            cursor: pointer;
            border: none;
            box-shadow: var(--shadow-soft);
        }

        /* Responsive design */
        @media (max-width: 768px) {
            .container {
                padding: 15px;
            }

            .header {
                padding: 20px;
            }

            .header h1 {
                font-size: 2rem;
            }

            .tabs {
                flex-wrap: wrap;
                gap: 4px;
            }

            .tab {
                padding: 10px 16px;
                font-size: 13px;
            }

            .tab-content {
                padding: 20px;
            }

            .sensor-grid {
                grid-template-columns: 1fr;
            }

            .button-grid {
                grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
            }
        }

        /* Loading animation */
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .loading {
            animation: pulse 2s infinite;
        }

        /* Custom scrollbar */
        ::-webkit-scrollbar {
            width: 8px;
        }

        ::-webkit-scrollbar-track {
            background: var(--secondary-bg);
        }

        ::-webkit-scrollbar-thumb {
            background: var(--border-color);
            border-radius: 4px;
        }

        ::-webkit-scrollbar-thumb:hover {
            background: var(--accent-blue);
        }

        /* Toggle Switch */
        .toggle-switch {
            position: relative;
            display: inline-block;
            width: 50px;
            height: 24px;
        }

        .toggle-switch input {
            opacity: 0;
            width: 0;
            height: 0;
        }

        .toggle-slider {
            position: absolute;
            cursor: pointer;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-color: var(--border-color);
            transition: 0.3s;
            border-radius: 24px;
        }

        .toggle-slider:before {
            position: absolute;
            content: "";
            height: 18px;
            width: 18px;
            left: 3px;
            bottom: 3px;
            background-color: white;
            transition: 0.3s;
            border-radius: 50%;
        }

        input:checked + .toggle-slider {
            background: var(--gradient-primary);
        }

        input:checked + .toggle-slider:before {
            transform: translateX(26px);
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚀 Autonomous Mobile Manipulator Control</h1>
            <p>Advanced real-time control interface for autonomous navigation, precision manipulation, and comprehensive sensor monitoring</p>
            <div style="display: flex; gap: 20px; margin-top: 20px; font-size: 14px;">
                <span style="display: flex; align-items: center; gap: 8px;">
                    <div style="width: 8px; height: 8px; background: #10b981; border-radius: 50%;"></div>
                    System Online
                </span>
                <span style="display: flex; align-items: center; gap: 8px;">
                    <div style="width: 8px; height: 8px; background: #3b82f6; border-radius: 50%;"></div>
                    ROS2 Active
                </span>
                <span style="display: flex; align-items: center; gap: 8px;">
                    <div style="width: 8px; height: 8px; background: #8b5cf6; border-radius: 50%;"></div>
                    Mega Connected
                </span>
            </div>
        </div>

        <div class="tabs">
            <button class="tab active" onclick="showTab('dashboard')">
                <span style="display: flex; align-items: center; gap: 8px;">
                    📊 Dashboard
                </span>
            </button>
            <button class="tab" onclick="showTab('movement')">
                <span style="display: flex; align-items: center; gap: 8px;">
                    🏃 Movement
                </span>
            </button>
            <button class="tab" onclick="showTab('manipulation')">
                <span style="display: flex; align-items: center; gap: 8px;">
                    🤖 Manipulation
                </span>
            </button>
            <button class="tab" onclick="showTab('pathplanning')">
                <span style="display: flex; align-items: center; gap: 8px;">
                    🗺️ Path Planning
                </span>
            </button>
            <button class="tab" onclick="showTab('sensors')">
                <span style="display: flex; align-items: center; gap: 8px;">
                    📡 Sensors
                </span>
            </button>
            <button class="tab" onclick="showTab('serial')">
                <span style="display: flex; align-items: center; gap: 8px;">
                    🔧 Direct Serial
                </span>
            </button>
            <button class="tab" onclick="showTab('serialmonitor')">
                <span style="display: flex; align-items: center; gap: 8px;">
                    📟 Serial Monitor
                </span>
            </button>
        </div>

        <!-- Dashboard Tab -->
        <div id="dashboard" class="tab-content">
            <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 30px; margin-bottom: 30px;">
                <!-- System Status Card -->
                <div class="control-group">
                    <h3 style="display: flex; align-items: center; gap: 10px;">
                        <span style="font-size: 1.5rem;">🖥️</span>
                        System Status
                    </h3>
                    <div id="system-status" class="status info" style="margin: 0;">
                        <div style="display: flex; justify-content: space-between; margin-bottom: 8px;">
                            <strong>Status:</strong>
                            <span id="status-text" style="font-weight: 600;">Initializing...</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin-bottom: 8px;">
                            <strong>Mega:</strong>
                            <div style="display: flex; align-items: center; gap: 8px;">
                                <div id="mega-connection-dot" style="width: 8px; height: 8px; border-radius: 50%; background: var(--text-muted);"></div>
                                <span id="mega-status" style="font-weight: 600;">Checking...</span>
                                <button id="mega-reconnect-btn" onclick="reconnectMega()" class="btn btn-sm" style="display: none; padding: 2px 8px; font-size: 12px;">🔄</button>
                            </div>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin-bottom: 8px;">
                            <strong>Port:</strong>
                            <span id="mega-port" style="font-weight: 600; font-family: monospace;">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between;">
                            <strong>ROS2 Services:</strong>
                            <span id="ros2-status" style="font-weight: 600;">Checking...</span>
                        </div>
                    </div>
                </div>

                <!-- Current Position Card -->
                <div class="control-group">
                    <h3 style="display: flex; align-items: center; gap: 10px;">
                        <span style="font-size: 1.5rem;">📍</span>
                        Current Position
                    </h3>
                    <div id="current-position-display" style="font-family: 'JetBrains Mono', monospace; font-size: 14px; color: var(--text-secondary);">
                        <div style="display: flex; justify-content: space-between; margin-bottom: 5px;">
                            <span>X:</span>
                            <span id="pos-x">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin-bottom: 5px;">
                            <span>Y:</span>
                            <span id="pos-y">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin-bottom: 5px;">
                            <span>Z:</span>
                            <span id="pos-z">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between;">
                            <span>Heading:</span>
                            <span id="pos-heading">--°</span>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Quick Actions -->
            <div class="control-group">
                <h3 style="display: flex; align-items: center; gap: 10px;">
                    <span style="font-size: 1.5rem;">⚡</span>
                    Quick Actions
                </h3>
                <div class="button-grid" style="grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));">
                    <button class="btn btn-danger" onclick="emergencyStop()" style="background: var(--gradient-danger);">
                        🚨 Emergency Stop
                    </button>
                    <button class="btn btn-warning" onclick="homeServos()" style="background: var(--gradient-warning); color: #1f2937;">
                        🏠 Home Servos
                    </button>
                    <button class="btn btn-info" onclick="testLimitSwitches()">
                        🔍 Test Limit Switches
                    </button>
                    <button class="btn btn-success" onclick="refreshSensors()">
                        🔄 Refresh Sensors
                    </button>
                </div>
            </div>

            <!-- Robot Log -->
            <div class="control-group">
                <h3 style="display: flex; align-items: center; gap: 10px;">
                    <span style="font-size: 1.5rem;">📝</span>
                    Robot Log
                </h3>
                <div id="robot-log" class="log-container" style="font-size: 12px;">
                    System initialized...<br>
                    Waiting for connections...
                </div>
            </div>
        </div>

        <!-- Movement Tab -->
        <div id="movement" class="tab-content hidden">
            <div style="margin-bottom: 30px;">
                <h2 style="display: flex; align-items: center; gap: 15px; margin: 0 0 20px 0;">
                    <span style="font-size: 2rem;">🏃</span>
                    Robot Movement Control
                </h2>
                <p style="color: var(--text-secondary); margin: 0;">Precise control over robot locomotion and navigation systems</p>
            </div>

            <!-- Speed and Turbo Controls -->
            <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 30px;">
                <div class="control-group">
                    <label for="speed-slider" style="font-size: 16px; font-weight: 600; margin-bottom: 15px;">
                        ⚡ Speed Control
                    </label>
                    <div style="display: flex; align-items: center; gap: 15px; margin-bottom: 15px;">
                        <input type="range" id="speed-slider" min="0" max="100" value="50" oninput="updateSpeedDisplay()" style="flex: 1;">
                        <span id="speed-display" style="font-size: 18px; font-weight: 700; color: var(--accent-blue); min-width: 60px;">50%</span>
                    </div>
                    <button class="btn btn-info" onclick="setSpeed()" style="width: 100%;">
                        ✅ Apply Speed
                    </button>
                </div>

                <div class="control-group">
                    <label style="font-size: 16px; font-weight: 600; margin-bottom: 15px;">
                        🚀 Performance Mode
                    </label>
                    <div style="display: flex; align-items: center; justify-content: space-between; margin-bottom: 15px;">
                        <span style="color: var(--text-secondary);">Turbo Mode:</span>
                        <div id="turbo-indicator" style="width: 12px; height: 12px; border-radius: 50%; background: var(--text-muted); transition: all 0.3s ease;"></div>
                    </div>
                    <button class="btn btn-warning" onclick="toggleTurbo()" style="width: 100%; background: var(--gradient-warning); color: #1f2937;">
                        🚀 Toggle Turbo
                    </button>
                </div>
            </div>

            <!-- Movement Controls -->
            <div class="control-group">
                <h3 style="display: flex; align-items: center; gap: 10px; font-size: 18px;">
                    <span style="font-size: 1.5rem;">🎮</span>
                    Movement Controls
                </h3>
                <div class="button-grid" style="grid-template-columns: repeat(3, 1fr); max-width: 500px; margin: 0 auto;">
                    <div></div>
                    <button class="btn btn-primary" onclick="moveRobot('forward')" style="height: 60px;">
                        <div style="font-size: 24px;">⬆️</div>
                        <div style="font-size: 12px; margin-top: 5px;">Forward</div>
                    </button>
                    <div></div>

                    <button class="btn btn-primary" onclick="turnRobot('left')" style="height: 60px;">
                        <div style="font-size: 24px;">⬅️</div>
                        <div style="font-size: 12px; margin-top: 5px;">Left</div>
                    </button>

                    <button class="btn btn-danger" onclick="stopRobot()" style="height: 60px; background: var(--gradient-danger);">
                        <div style="font-size: 24px;">⏹️</div>
                        <div style="font-size: 12px; margin-top: 5px;">STOP</div>
                    </button>

                    <button class="btn btn-primary" onclick="turnRobot('right')" style="height: 60px;">
                        <div style="font-size: 24px;">➡️</div>
                        <div style="font-size: 12px; margin-top: 5px;">Right</div>
                    </button>

                    <div></div>
                    <button class="btn btn-primary" onclick="moveRobot('backward')" style="height: 60px;">
                        <div style="font-size: 24px;">⬇️</div>
                        <div style="font-size: 12px; margin-top: 5px;">Backward</div>
                    </button>
                    <div></div>
                </div>
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

        <!-- Path Planning Tab -->
        <div id="pathplanning" class="tab-content hidden">
            <h2>Path Planning & Movement Sequences</h2>

            <h3>Predefined Movement Sets</h3>
            <div class="button-grid">
                <button class="btn btn-primary" onclick="executeMovementSet('lawnmower')">🌾 Lawn Mower</button>
                <button class="btn btn-primary" onclick="executeMovementSet('spiral')">🌀 Spiral Search</button>
                <button class="btn btn-primary" onclick="executeMovementSet('boundary')">🔲 Boundary Follow</button>
                <button class="btn btn-primary" onclick="executeMovementSet('zigzag')">⚡ Zigzag Scan</button>
                <button class="btn btn-primary" onclick="executeMovementSet('figure8')">∞ Figure-8</button>
                <button class="btn btn-primary" onclick="executeMovementSet('circle')">⭕ Circle</button>
                <button class="btn btn-success" onclick="executeMovementSet('return_home')">🏠 Return Home</button>
                <button class="btn btn-danger" onclick="stopSequence()">⏹️ Stop Sequence</button>
            </div>

            <h3>Movement Sequence Editor</h3>
            <div class="control-group">
                <label for="sequence-name">Sequence Name:</label>
                <input type="text" id="sequence-name" placeholder="Enter sequence name">
            </div>

            <div class="control-group">
                <label>Add Commands to Sequence:</label>
                <div class="button-grid">
                    <!-- Movement Commands -->
                    <button class="btn btn-secondary" onclick="addToSequence('f')">⬆️ Forward</button>
                    <button class="btn btn-secondary" onclick="addToSequence('r')">➡️ Right</button>
                    <button class="btn btn-secondary" onclick="addToSequence('b')">⬇️ Backward</button>
                    <button class="btn btn-secondary" onclick="addToSequence('l')">⬅️ Left</button>
                    <button class="btn btn-secondary" onclick="addToSequence('q')">↖️ F-Left</button>
                    <button class="btn btn-secondary" onclick="addToSequence('e')">↗️ F-Right</button>
                    <button class="btn btn-secondary" onclick="addToSequence('z')">↙️ B-Left</button>
                    <button class="btn btn-secondary" onclick="addToSequence('x')">↘️ B-Right</button>
                    <button class="btn btn-warning" onclick="addToSequence('s')">⏹️ Stop</button>
                </div>

                <div class="button-grid">
                    <!-- Control Commands -->
                    <button class="btn btn-info" onclick="addToSequence('p')">📊 Status</button>
                    <button class="btn btn-warning" onclick="addToSequence('5')">5️⃣ Speed 50%</button>
                    <button class="btn btn-warning" onclick="addToSequence('7')">7️⃣ Speed 70%</button>
                    <button class="btn btn-warning" onclick="addToSequence('0')">🔟 Speed 100%</button>
                    <button class="btn btn-success" onclick="addToSequence('u')">⬆️ Lift Up</button>
                    <button class="btn btn-danger" onclick="addToSequence('d')">⬇️ Lift Down</button>
                </div>
            </div>

            <div class="control-group">
                <label>Current Sequence:</label>
                <div id="sequence-display" class="log-container" style="min-height: 100px;">
                    <!-- Sequence commands will be displayed here -->
                </div>
                <div class="button-grid">
                    <button class="btn btn-success" onclick="saveSequence()">💾 Save Sequence</button>
                    <button class="btn btn-info" onclick="loadSequence()">📂 Load Sequence</button>
                    <button class="btn btn-warning" onclick="executeCustomSequence()">▶️ Execute Sequence</button>
                    <button class="btn btn-danger" onclick="clearSequence()">🗑️ Clear Sequence</button>
                </div>
            </div>

            <h3>Saved Sequences</h3>
            <div id="saved-sequences" class="log-container">
                <!-- Saved sequences will be listed here -->
            </div>

            <h3>Waypoint Navigation</h3>

            <!-- Map Visualization -->
            <div class="control-group">
                <label>Map Visualization (50px = 1m)</label>
                <div id="map-container">
                    <p>Loading map...</p>
                </div>
                <div class="button-grid">
                    <button class="btn btn-info" onclick="clearMapCanvas()">🗑️ Clear Map</button>
                    <button class="btn btn-secondary" onclick="loadMap()">🔄 Reload Map</button>
                </div>
            </div>

            <!-- Waypoint Examples -->
            <div class="control-group">
                <label>Waypoint Examples:</label>
                <div class="button-grid">
                    <button class="btn btn-info" onclick="loadWaypointExample('square')">▢ Square Path (4m)</button>
                    <button class="btn btn-info" onclick="loadWaypointExample('triangle')">△ Triangle Path (3m)</button>
                    <button class="btn btn-info" onclick="loadWaypointExample('circle')">⭕ Circle Path (5m)</button>
                    <button class="btn btn-info" onclick="loadWaypointExample('figure8')">∞ Figure-8 Path</button>
                    <button class="btn btn-info" onclick="loadWaypointExample('inspection')">🔍 Room Inspection</button>
                    <button class="btn btn-info" onclick="loadWaypointExample('delivery')">📦 Delivery Route</button>
                </div>
            </div>

            <!-- Manual Waypoint Entry -->
            <div class="control-group">
                <label>Add Waypoint (ENU Coordinates):</label>
                <div class="input-group">
                    <label>Easting (X) meters:</label>
                    <input type="number" id="waypoint-x" step="0.1" placeholder="East coordinate">
                </div>
                <div class="input-group">
                    <label>Northing (Y) meters:</label>
                    <input type="number" id="waypoint-y" step="0.1" placeholder="North coordinate">
                </div>
                <div class="input-group">
                    <label>Up (Z) meters:</label>
                    <input type="number" id="waypoint-z" step="0.1" placeholder="Up coordinate" value="0">
                </div>
                <button class="btn btn-primary" onclick="addWaypoint()">📍 Add Waypoint</button>
                <button class="btn btn-success" onclick="navigateWaypoints()">🧭 Navigate Waypoints</button>
                <button class="btn btn-danger" onclick="clearWaypoints()">🗑️ Clear Waypoints</button>
            </div>

            <!-- Current Position Display -->
            <div class="control-group">
                <label>Current Position (ENU from IMU):</label>
                <div class="input-group">
                    <span>E: <span id="current-x">0.0</span>m</span>
                    <span>N: <span id="current-y">0.0</span>m</span>
                    <span>U: <span id="current-z">0.0</span>m</span>
                    <span>Heading: <span id="current-heading">0.0</span>°</span>
                </div>
                <button class="btn btn-info" onclick="updateCurrentPosition()">📡 Update Position</button>
                <button class="btn btn-secondary" onclick="setCurrentAsWaypoint()">📍 Set Current as Waypoint</button>
                <button class="btn btn-warning" onclick="resetOrigin()">🏠 Reset Origin (0,0,0)</button>
            </div>

            <!-- Waypoint List -->
            <div class="control-group">
                <label>Waypoint List:</label>
                <div id="waypoint-list" class="log-container" style="min-height: 80px;">
                    <!-- Waypoints will be listed here -->
                </div>
                <div class="button-grid">
                    <button class="btn btn-warning" onclick="reverseWaypoints()">🔄 Reverse Route</button>
                    <button class="btn btn-info" onclick="optimizeWaypoints()">⚡ Optimize Route</button>
                    <button class="btn btn-secondary" onclick="saveWaypointRoute()">💾 Save Route</button>
                    <button class="btn btn-secondary" onclick="loadWaypointRoute()">📂 Load Route</button>
                </div>
            </div>

            <!-- Waypoint Examples Info -->
            <div class="control-group">
                <label>Waypoint Navigation Guide:</label>
                <div id="example-info" class="log-container" style="font-size: 12px; line-height: 1.4;">
                    <strong>📐 Coordinate System:</strong> East=X, North=Y, Up=Z (ENU)<br>
                    <strong>🚀 Navigation:</strong> Robot moves between waypoints in order<br>
                    <strong>⚡ Path Planning:</strong> A* algorithm avoids obstacles automatically<br><br>

                    <strong>🎯 Example Scenarios:</strong><br>
                    <strong>• Boundary Patrol:</strong> Square path around perimeter<br>
                    <strong>• Search Pattern:</strong> Spiral or zigzag for area coverage<br>
                    <strong>• Delivery Route:</strong> Multiple stops with return to start<br>
                    <strong>• Inspection Tour:</strong> Systematic coverage of workspace<br><br>

                    <strong>💡 Pro Tips:</strong><br>
                    • Start waypoints from robot's current position (0,0,0)<br>
                    • Use room dimensions for coordinate planning<br>
                    • Save successful routes for reuse<br>
                    • Test with "Stop Sequence" for safety
                </div>
            </div>

            <!-- Tutorial Section -->
            <div class="control-group">
                <label>Quick Start Tutorial:</label>
                <div class="log-container" style="font-size: 12px; line-height: 1.4;">
                    <strong>Step 1:</strong> Click "Square Path" example<br>
                    <strong>Step 2:</strong> Click "Navigate Waypoints"<br>
                    <strong>Step 3:</strong> Watch robot move 4m square pattern<br>
                    <strong>Step 4:</strong> Use "Stop Sequence" to halt if needed<br><br>

                    <strong>Custom Route:</strong><br>
                    1. Clear waypoints<br>
                    2. Add points: (0,0) → (2,0) → (2,2) → (0,2)<br>
                    3. Navigate and observe rectangular path<br><br>

                    <strong>📋 Real-World Examples:</strong><br><br>

                    <strong>🏢 Office Inspection:</strong><br>
                    Start (0,0) → Reception (3,0) → Meeting Room (3,4) → Kitchen (0,4) → Start<br>
                    <em>Use: Regular facility monitoring</em><br><br>

                    <strong>🏭 Warehouse Inventory:</strong><br>
                    Dock (0,0) → Aisle1 (5,0) → Aisle2 (5,8) → Aisle3 (5,16) → Shipping (0,16)<br>
                    <em>Use: Automated stock checking</em><br><br>

                    <strong>🏥 Hospital Rounds:</strong><br>
                    Nurses Station (0,0) → Room101 (4,0) → Room102 (4,3) → Room103 (4,6) → Station<br>
                    <em>Use: Medical supply delivery</em><br><br>

                    <strong>🏪 Store Cleaning:</strong><br>
                    Storage (0,0) → Produce (6,0) → Dairy (6,4) → Bakery (6,8) → Checkout (0,8)<br>
                    <em>Use: Automated floor cleaning</em><br><br>

                    <strong>🔍 Search & Rescue:</strong><br>
                    Base (0,0) → Grid1 (10,0) → Grid2 (10,10) → Grid3 (0,10) → Spiral outward<br>
                    <em>Use: Systematic area search</em><br><br>

                    <strong>📐 Coordinate Planning Tips:</strong><br>
                    • Measure room dimensions in meters<br>
                    • Use tape measure for accuracy<br>
                    • Mark waypoints on floor plan first<br>
                    • Test small routes before large ones<br>
                    • Save successful routes for reuse
                </div>
            </div>
        </div>

        <!-- Sensors Tab -->
        <div id="sensors" class="tab-content hidden">
            <div style="margin-bottom: 30px;">
                <h2 style="display: flex; align-items: center; gap: 15px; margin: 0 0 20px 0;">
                    <span style="font-size: 2rem;">📡</span>
                    Sensor Monitoring
                </h2>
                <p style="color: var(--text-secondary); margin: 0;">Real-time environmental sensing and spatial awareness</p>
            </div>

            <!-- Control Panel -->
            <div class="control-group" style="margin-bottom: 30px;">
                <div style="display: flex; justify-content: space-between; align-items: center;">
                    <div>
                        <h3 style="margin: 0 0 5px 0; font-size: 16px;">Sensor Control</h3>
                        <p style="margin: 0; color: var(--text-muted); font-size: 14px;">Monitor and refresh sensor data</p>
                    </div>
                    <button class="btn btn-info" onclick="refreshSensors()" style="display: flex; align-items: center; gap: 8px;">
                        <span style="font-size: 16px;">🔄</span>
                        Refresh All
                    </button>
                </div>
            </div>

            <!-- IR Distance Sensors -->
            <div class="control-group">
                <h3 style="display: flex; align-items: center; gap: 10px; margin-bottom: 20px;">
                    <span style="font-size: 1.5rem;">👁️</span>
                    IR Distance Sensors
                    <span style="background: var(--gradient-primary); color: white; padding: 2px 8px; border-radius: 12px; font-size: 12px; font-weight: 500;">6 Sensors</span>
                </h3>
                <div class="sensor-grid" id="ir-sensors">
                    <div class="sensor-card" style="text-align: center; color: var(--text-muted);">
                        <div style="font-size: 48px; margin-bottom: 10px;">📡</div>
                        <div>Loading IR sensors...</div>
                    </div>
                </div>
            </div>

            <!-- Ultrasonic Sensors -->
            <div class="control-group">
                <h3 style="display: flex; align-items: center; gap: 10px; margin-bottom: 20px;">
                    <span style="font-size: 1.5rem;">🌊</span>
                    Ultrasonic Sensors
                    <span style="background: var(--gradient-success); color: white; padding: 2px 8px; border-radius: 12px; font-size: 12px; font-weight: 500;">2 Sensors</span>
                </h3>
                <div class="sensor-grid" id="ultrasonic-sensors">
                    <div class="sensor-card" style="text-align: center; color: var(--text-muted);">
                        <div style="font-size: 48px; margin-bottom: 10px;">📡</div>
                        <div>Loading ultrasonic sensors...</div>
                    </div>
                </div>
            </div>

            <!-- IMU Data -->
            <div class="control-group">
                <h3 style="display: flex; align-items: center; gap: 10px; margin-bottom: 20px;">
                    <span style="font-size: 1.5rem;">🧭</span>
                    IMU & Orientation
                    <span style="background: var(--gradient-purple); color: white; padding: 2px 8px; border-radius: 12px; font-size: 12px; font-weight: 500;">9-DOF</span>
                </h3>
                <div class="sensor-grid" id="imu-data">
                    <div class="sensor-card" style="text-align: center; color: var(--text-muted);">
                        <div style="font-size: 48px; margin-bottom: 10px;">🧭</div>
                        <div>Loading IMU data...</div>
                    </div>
                </div>
            </div>

            <!-- Sensor Health Status -->
            <div class="control-group">
                <h3 style="display: flex; align-items: center; gap: 10px; margin-bottom: 20px;">
                    <span style="font-size: 1.5rem;">💚</span>
                    System Health
                </h3>
                <div id="sensor-health" class="status info">
                    <div style="display: flex; align-items: center; gap: 15px;">
                        <div style="display: flex; align-items: center; gap: 8px;">
                            <div style="width: 12px; height: 12px; background: #10b981; border-radius: 50%;"></div>
                            <span>Sensors: <strong>Online</strong></span>
                        </div>
                        <div style="display: flex; align-items: center; gap: 8px;">
                            <div style="width: 12px; height: 12px; background: #10b981; border-radius: 50%;"></div>
                            <span>Mega: <strong>Connected</strong></span>
                        </div>
                        <div style="display: flex; align-items: center; gap: 8px;">
                            <div style="width: 12px; height: 12px; background: #10b981; border-radius: 50%;"></div>
                            <span>ROS2: <strong>Active</strong></span>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Direct Serial Tab -->
        <div id="serial" class="tab-content hidden">
            <div style="margin-bottom: 30px;">
                <h2 style="display: flex; align-items: center; gap: 15px; margin: 0 0 20px 0;">
                    <span style="font-size: 2rem;">🔧</span>
                    Direct Serial Communication
                </h2>
                <p style="color: var(--text-secondary); margin: 0;">Low-level Arduino Mega control bypassing ROS2 abstraction layer</p>
            </div>

            <!-- Command Input -->
            <div class="control-group">
                <label for="serial-command" style="font-size: 16px; font-weight: 600; margin-bottom: 15px; display: block;">
                    🔍 Direct Command Input
                </label>
                <div style="display: flex; gap: 15px; align-items: center;">
                    <div style="flex: 1; position: relative;">
                        <input type="text" id="serial-command"
                               placeholder="Enter command (e.g., f, s, p, ls, se, sd)"
                               style="width: 100%; padding-right: 50px;">
                        <div style="position: absolute; right: 15px; top: 50%; transform: translateY(-50%); color: var(--text-muted); font-size: 12px;">
                            Serial
                        </div>
                    </div>
                    <button class="btn btn-success" onclick="sendSerialCommand()" style="display: flex; align-items: center; gap: 8px;">
                        <span style="font-size: 16px;">📤</span>
                        Send
                    </button>
                </div>
                <div class="description" style="margin-top: 10px;">
                    Use single character commands or multi-character sequences. Check COMMANDS.md for full reference.
                </div>
            </div>

            <!-- Quick Commands -->
            <div class="control-group">
                <h3 style="display: flex; align-items: center; gap: 10px; margin-bottom: 20px;">
                    <span style="font-size: 1.5rem;">⚡</span>
                    Quick Commands
                    <span style="background: var(--gradient-primary); color: white; padding: 2px 8px; border-radius: 12px; font-size: 12px; font-weight: 500;">Essential</span>
                </h3>
                <div class="button-grid" style="grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));">
                    <button class="btn btn-info" onclick="sendQuickCommand('p')" style="display: flex; align-items: center; justify-content: center; gap: 8px;">
                        <span style="font-size: 16px;">📊</span>
                        <div style="text-align: left;">
                            <div style="font-weight: 600;">Status</div>
                            <div style="font-size: 12px; opacity: 0.8;">(p)</div>
                        </div>
                    </button>
                    <button class="btn btn-warning" onclick="sendQuickCommand('ls')" style="display: flex; align-items: center; justify-content: center; gap: 8px;">
                        <span style="font-size: 16px;">🔍</span>
                        <div style="text-align: left;">
                            <div style="font-weight: 600;">Limit Switch Test</div>
                            <div style="font-size: 12px; opacity: 0.8;">(ls)</div>
                        </div>
                    </button>
                    <button class="btn btn-success" onclick="sendQuickCommand('se')" style="display: flex; align-items: center; justify-content: center; gap: 8px;">
                        <span style="font-size: 16px;">🛡️</span>
                        <div style="text-align: left;">
                            <div style="font-weight: 600;">Enable Safety</div>
                            <div style="font-size: 12px; opacity: 0.8;">(se)</div>
                        </div>
                    </button>
                    <button class="btn btn-danger" onclick="sendQuickCommand('sd')" style="display: flex; align-items: center; justify-content: center; gap: 8px;">
                        <span style="font-size: 16px;">⚠️</span>
                        <div style="text-align: left;">
                            <div style="font-weight: 600;">Disable Safety</div>
                            <div style="font-size: 12px; opacity: 0.8;">(sd)</div>
                        </div>
                    </button>
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
                <!-- Cardinal Directions -->
                <button class="btn btn-primary" onclick="sendQuickCommand('f')">⬆️ Forward (f)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('b')">⬇️ Backward (b)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('l')">⬅️ Strafe Left (l)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('r')">➡️ Strafe Right (r)</button>

                <!-- Diagonal Directions -->
                <button class="btn btn-primary" onclick="sendQuickCommand('q')">↖️ Forward-Left (q)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('e')">↗️ Forward-Right (e)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('z')">↙️ Backward-Left (z)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('x')">↘️ Backward-Right (x)</button>

                <!-- Rotation -->
                <button class="btn btn-primary" onclick="sendQuickCommand('c')">🔄 Clockwise (c)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('w')">🔄 Counter-clockwise (w)</button>

                <!-- Turning -->
                <button class="btn btn-primary" onclick="sendQuickCommand('t')">↺ Turn Left (t)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('y')">↻ Turn Right (y)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('a')">🌀 Arc Left (a)</button>
                <button class="btn btn-primary" onclick="sendQuickCommand('j')">🌀 Arc Right (j)</button>

                <!-- Stop -->
                <button class="btn btn-danger" onclick="sendQuickCommand('s')">⏹️ Stop (s)</button>
            </div>

            <h3>Control Commands</h3>
            <div class="button-grid">
                <button class="btn btn-info" onclick="sendQuickCommand('p')">📊 Status (p)</button>
                <button class="btn btn-warning" onclick="sendQuickCommand('v')">🚨 Emergency Stop (v)</button>
                <button class="btn btn-secondary" onclick="sendQuickCommand('o')">🚀 Turbo Toggle (o)</button>
            </div>

            <h3>Speed Control</h3>
            <div class="button-grid">
                <button class="btn btn-secondary" onclick="sendQuickCommand('5')">5️⃣ 50% (5)</button>
                <button class="btn btn-secondary" onclick="sendQuickCommand('6')">6️⃣ 60% (6)</button>
                <button class="btn btn-secondary" onclick="sendQuickCommand('7')">7️⃣ 70% (7)</button>
                <button class="btn btn-secondary" onclick="sendQuickCommand('8')">8️⃣ 80% (8)</button>
                <button class="btn btn-secondary" onclick="sendQuickCommand('9')">9️⃣ 90% (9)</button>
                <button class="btn btn-secondary" onclick="sendQuickCommand('0')">🔟 100% (0)</button>
            </div>

            <h3>Lifter Commands</h3>
            <div class="button-grid">
                <button class="btn btn-success" onclick="sendQuickCommand('u')">⬆️ Lift Up (u)</button>
                <button class="btn btn-danger" onclick="sendQuickCommand('d')">⬇️ Lift Down (d)</button>
            </div>

            <h3>Testing Commands</h3>
            <div class="button-grid">
                <button class="btn btn-warning" onclick="sendQuickCommand('1')">1️⃣ Test Motor 1 (lifter)</button>
                <button class="btn btn-warning" onclick="sendQuickCommand('2')">2️⃣ Test Motor 2 (FR)</button>
                <button class="btn btn-warning" onclick="sendQuickCommand('3')">3️⃣ Test Motor 3 (FL)</button>
                <button class="btn btn-warning" onclick="sendQuickCommand('4')">4️⃣ Test Motor 4 (Back)</button>
                <button class="btn btn-warning" onclick="sendQuickCommand('g')">∞ Figure-8 (g)</button>
                <button class="btn btn-warning" onclick="sendQuickCommand('h')">🔄 Continuous Rotation (h)</button>
            </div>

            <h3>Safety Commands</h3>
            <div class="button-grid">
                <button class="btn btn-success" onclick="sendQuickCommand('se')">🛡️ Enable Safety (se)</button>
                <button class="btn btn-danger" onclick="sendQuickCommand('sd')">⚠️ Disable Safety (sd)</button>
            </div>

            <h3>Sensor Commands</h3>
            <div class="button-grid">
                <button class="btn btn-info" onclick="sendQuickCommand('sr')">📡 Sensor Readings (sr)</button>
                <button class="btn btn-info" onclick="sendQuickCommand('ls')">🔍 Limit Switch Test (ls)</button>
            </div>

            <h3>Publishing Control</h3>
            <div class="button-grid">
                <button class="btn btn-success" onclick="sendQuickCommand('spe')">📤 Enable Publishing (spe)</button>
                <button class="btn btn-danger" onclick="sendQuickCommand('spd')">📥 Disable Publishing (spd)</button>
            </div>

            <h3>Individual Wheel Control</h3>
            <div class="control-group">
                <label>Wheel Speed Commands (w[wheel][speed]):</label>
                <div class="input-group">
                    <label>Wheel:</label>
                    <select id="wheel-select-serial">
                        <option value="0">0 - Lifter Motor</option>
                        <option value="1">1 - Front Right</option>
                        <option value="2">2 - Front Left</option>
                        <option value="3">3 - Back Motor</option>
                    </select>
                </div>
                <div class="input-group">
                    <label>Speed (-100 to 100):</label>
                    <input type="number" id="wheel-speed-serial" min="-100" max="100" value="50" step="10">
                </div>
                <button class="btn btn-info" onclick="sendWheelCommand()">Set Wheel Speed</button>
                <button class="btn btn-danger" onclick="sendQuickCommand('wstop')">Stop All Wheels (wstop)</button>
            </div>

            <h3>Command Log</h3>
            <div class="log-container" id="serial-log">
                <!-- Serial command log will be populated here -->
            </div>
        </div>

        <!-- Serial Monitor Tab -->
        <div id="serialmonitor" class="tab-content hidden">
            <div style="margin-bottom: 30px;">
                <h2 style="display: flex; align-items: center; gap: 15px; margin: 0 0 20px 0;">
                    <span style="font-size: 2rem;">📟</span>
                    Serial Monitor
                </h2>
                <p style="color: var(--text-secondary); margin: 0;">Real-time raw serial data stream from Arduino Mega microcontroller</p>
            </div>

            <!-- Serial Monitor Controls -->
            <div class="control-group" style="margin-bottom: 30px;">
                <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px;">
                    <div>
                        <h3 style="margin: 0 0 5px 0; font-size: 16px;">Monitor Controls</h3>
                        <p style="margin: 0; color: var(--text-muted); font-size: 14px;">Control serial data monitoring and display</p>
                    </div>
                    <div style="display: flex; gap: 15px;">
                        <button class="btn btn-success" onclick="startSerialMonitor()" id="monitor-start-btn" style="display: flex; align-items: center; gap: 8px;">
                            <span style="font-size: 16px;">▶️</span>
                            Start Monitor
                        </button>
                        <button class="btn btn-danger" onclick="stopSerialMonitor()" id="monitor-stop-btn" style="display: none; flex: align-items: center; gap: 8px;">
                            <span style="font-size: 16px;">⏹️</span>
                            Stop Monitor
                        </button>
                        <button class="btn btn-info" onclick="clearSerialMonitor()" style="display: flex; align-items: center; gap: 8px;">
                            <span style="font-size: 16px;">🗑️</span>
                            Clear
                        </button>
                        <button class="btn btn-warning" onclick="testSerialMonitor()" style="display: flex; align-items: center; gap: 8px;">
                            <span style="font-size: 16px;">🔧</span>
                            Test
                        </button>
                        <button class="btn btn-secondary" onclick="forceShowData()" style="display: flex; align-items: center; gap: 8px;">
                            <span style="font-size: 16px;">💪</span>
                            Force Show
                        </button>
                    </div>
                </div>

                <!-- Monitor Settings -->
                <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px;">
                    <div>
                        <label for="monitor-autoscroll" style="font-weight: 500; margin-bottom: 8px; display: block;">Auto-scroll:</label>
                        <div style="display: flex; align-items: center; gap: 10px;">
                            <label class="toggle-switch">
                                <input type="checkbox" id="monitor-autoscroll" checked>
                                <span class="toggle-slider"></span>
                            </label>
                            <span style="color: var(--text-secondary); font-size: 14px;">Enabled</span>
                        </div>
                    </div>

                    <div>
                        <label for="monitor-timestamps" style="font-weight: 500; margin-bottom: 8px; display: block;">Timestamps:</label>
                        <div style="display: flex; align-items: center; gap: 10px;">
                            <label class="toggle-switch">
                                <input type="checkbox" id="monitor-timestamps" checked>
                                <span class="toggle-slider"></span>
                            </label>
                            <span style="color: var(--text-secondary); font-size: 14px;">Show timestamps</span>
                        </div>
                    </div>

                    <div>
                        <label for="monitor-filter" style="font-weight: 500; margin-bottom: 8px; display: block;">Filter:</label>
                        <select id="monitor-filter" style="width: 100%;">
                            <option value="all">All Data</option>
                            <option value="commands">Commands Only</option>
                            <option value="responses">Responses Only</option>
                            <option value="errors">Errors Only</option>
                            <option value="sensors">Sensor Data</option>
                        </select>
                    </div>

                    <div>
                        <label style="font-weight: 500; margin-bottom: 8px; display: block;">Connection Status:</label>
                        <div style="display: flex; align-items: center; gap: 10px;">
                            <div id="monitor-connection-status" style="width: 12px; height: 12px; border-radius: 50%; background: var(--text-muted); transition: all 0.3s ease;"></div>
                            <span id="monitor-connection-text" style="color: var(--text-secondary); font-size: 14px;">Disconnected</span>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Serial Data Display -->
            <div class="control-group">
                <h3 style="display: flex; align-items: center; gap: 10px; margin-bottom: 20px;">
                    <span style="font-size: 1.5rem;">📊</span>
                    Raw Serial Data Stream
                    <span id="data-rate-badge" style="background: var(--gradient-info); color: white; padding: 2px 8px; border-radius: 12px; font-size: 12px; font-weight: 500;">0 B/s</span>
                </h3>

                <div class="log-container" id="serial-monitor-output" style="font-family: 'JetBrains Mono', 'Fira Code', 'Courier New', monospace; font-size: 12px; line-height: 1.4; max-height: 600px;">
                    <div id="serial-monitor-placeholder" style="text-align: center; color: var(--text-muted); padding: 40px;">
                        <div style="font-size: 48px; margin-bottom: 15px;">📟</div>
                        <div id="serial-status-text">Serial monitor stopped. Click "Start Monitor" to begin receiving data.</div>
                        <div style="margin-top: 10px; font-size: 14px; opacity: 0.7;">Raw data from Arduino Mega will appear here in real-time.</div>
                        <div id="debug-info" style="margin-top: 20px; font-size: 12px; color: var(--accent-blue);"></div>
                    </div>
                </div>

                <!-- Data Statistics -->
                <div style="margin-top: 20px; padding: 15px; background: var(--card-bg); border-radius: 8px; border: 1px solid var(--border-color);">
                    <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 15px; font-size: 14px;">
                        <div>
                            <span style="color: var(--text-secondary);">Total Bytes:</span>
                            <span id="stats-total-bytes" style="margin-left: 8px; font-weight: 600; color: var(--accent-blue);">0</span>
                        </div>
                        <div>
                            <span style="color: var(--text-secondary);">Lines Received:</span>
                            <span id="stats-lines" style="margin-left: 8px; font-weight: 600; color: var(--accent-green);">0</span>
                        </div>
                        <div>
                            <span style="color: var(--text-secondary);">Errors Detected:</span>
                            <span id="stats-errors" style="margin-left: 8px; font-weight: 600; color: var(--accent-red);">0</span>
                        </div>
                        <div>
                            <span style="color: var(--text-secondary);">Runtime:</span>
                            <span id="stats-runtime" style="margin-left: 8px; font-weight: 600; color: var(--accent-purple);">00:00:00</span>
                        </div>
                    </div>
                </div>
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

            // Load map when Path Planning tab is activated
            if (tabName === 'pathplanning') {
                loadMap();
            }

            // Start serial monitor when Serial Monitor tab is activated
            if (tabName === 'serialmonitor') {
                console.log('Serial Monitor tab activated - starting monitor...');
                // Small delay to ensure tab content is visible
                setTimeout(() => {
                    console.log('Calling startSerialMonitor()...');
                    startSerialMonitor();
                }, 100);
            }

            // Stop serial monitor when leaving the tab
            if (currentTab === 'serialmonitor' && tabName !== 'serialmonitor') {
                stopSerialMonitor();
            }
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
            const indicator = document.getElementById('turbo-indicator');
            const currentColor = indicator.style.backgroundColor;

            // Toggle visual indicator
            if (currentColor === 'rgb(239, 68, 68)' || currentColor === '#ef4444') {
                indicator.style.backgroundColor = 'var(--text-muted)';
            } else {
                indicator.style.backgroundColor = '#ef4444';
                indicator.style.boxShadow = '0 0 10px rgba(239, 68, 68, 0.5)';
            }

            const result = await apiCall('/api/robot/turbo');
            if (!result || !result.success) {
                // Revert on error
                indicator.style.backgroundColor = 'var(--text-muted)';
                indicator.style.boxShadow = 'none';
            }
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

        async function sendWheelCommand() {
            const wheel = document.getElementById('wheel-select-serial').value;
            const speed = document.getElementById('wheel-speed-serial').value;
            const command = `w${wheel}${speed}`;
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

        // Path Planning Variables
        let currentSequence = [];
        let waypoints = [];
        let savedSequences = JSON.parse(localStorage.getItem('robotSequences') || '{}');

        // Map Rendering Variables
        let mapCanvas = null;
        let mapContext = null;
        let mapScale = 50; // 50 pixels = 1 meter
        let showGrid = true;
        let robotPosition = { x: 0, y: 0, heading: 0 };

        // Serial Monitor Variables
        let serialMonitorActive = false;
        let serialMonitorInterval = null;
        let serialMonitorStartTime = null;
        let fetchCount = 0;
        let serialStats = {
            totalBytes: 0,
            linesReceived: 0,
            errorsDetected: 0,
            lastDataRate: 0
        };

        // Add a simple status indicator
        let statusUpdateCount = 0;

        // Movement Sets
        async function executeMovementSet(setName) {
            showStatus('info', `Executing ${setName} movement set...`);

            const movementSets = {
                'lawnmower': [
                    'f', 'r', 'f', 'r', 'f', 'r', 'f', 'r', 'f', 's'  // Simple lawn mower pattern
                ],
                'spiral': [
                    'f', 'e', 'f', 'e', 'f', 'e', 'f', 's'  // Spiral outward
                ],
                'boundary': [
                    'f', 'r', 'f', 'r', 'f', 'r', 'f', 's'  // Follow boundary
                ],
                'zigzag': [
                    'f', 'r', 'f', 'l', 'f', 'r', 'f', 'l', 's'  // Zigzag pattern
                ],
                'figure8': [
                    'g'  // Use built-in figure-8 command
                ],
                'circle': [
                    'c', 'c', 'c', 's'  // Continuous rotation for circle
                ],
                'return_home': [
                    'b', 'b', 's'  // Simple return pattern
                ]
            };

            const sequence = movementSets[setName];
            if (sequence) {
                await executeSequence(sequence);
            }
        }

        async function executeSequence(commands) {
            for (const command of commands) {
                if (command === 's') {
                    // Stop command - wait a bit longer
                    await new Promise(resolve => setTimeout(resolve, 1000));
                } else {
                    // Regular command - execute and wait
                    await apiCall('/api/serial/send', { command });
                    await new Promise(resolve => setTimeout(resolve, 500));
                }
            }
            showStatus('success', 'Movement sequence completed');
        }

        async function stopSequence() {
            await apiCall('/api/serial/send', { command: 's' });
            showStatus('warning', 'Movement sequence stopped');
        }

        // Sequence Editor Functions
        function addToSequence(command) {
            currentSequence.push(command);
            updateSequenceDisplay();
        }

        function updateSequenceDisplay() {
            const display = document.getElementById('sequence-display');
            display.innerHTML = '';

            if (currentSequence.length === 0) {
                display.innerHTML = '<em>No commands in sequence</em>';
                return;
            }

            const commandNames = {
                'f': 'Forward', 'b': 'Backward', 'l': 'Left', 'r': 'Right',
                'q': 'Forward-Left', 'e': 'Forward-Right', 'z': 'Backward-Left', 'x': 'Backward-Right',
                'c': 'Clockwise', 'w': 'Counter-clockwise', 't': 'Turn Left', 'y': 'Turn Right',
                'a': 'Arc Left', 'j': 'Arc Right', 's': 'Stop', 'p': 'Status',
                '5': 'Speed 50%', '7': 'Speed 70%', '0': 'Speed 100%',
                'u': 'Lift Up', 'd': 'Lift Down'
            };

            currentSequence.forEach((cmd, index) => {
                const cmdName = commandNames[cmd] || cmd;
                const step = document.createElement('div');
                step.innerHTML = `${index + 1}. ${cmdName} (${cmd})`;
                display.appendChild(step);
            });
        }

        function saveSequence() {
            const name = document.getElementById('sequence-name').value.trim();
            if (!name) {
                showStatus('error', 'Please enter a sequence name');
                return;
            }

            if (currentSequence.length === 0) {
                showStatus('error', 'Sequence is empty');
                return;
            }

            savedSequences[name] = [...currentSequence];
            localStorage.setItem('robotSequences', JSON.stringify(savedSequences));

            updateSavedSequences();
            showStatus('success', `Sequence "${name}" saved`);
            document.getElementById('sequence-name').value = '';
        }

        function loadSequence() {
            const name = document.getElementById('sequence-name').value.trim();
            if (!name || !savedSequences[name]) {
                showStatus('error', 'Sequence not found');
                return;
            }

            currentSequence = [...savedSequences[name]];
            updateSequenceDisplay();
            showStatus('success', `Sequence "${name}" loaded`);
        }

        function executeCustomSequence() {
            if (currentSequence.length === 0) {
                showStatus('error', 'No sequence to execute');
                return;
            }

            showStatus('info', 'Executing custom sequence...');
            executeSequence(currentSequence);
        }

        function clearSequence() {
            currentSequence = [];
            updateSequenceDisplay();
            showStatus('info', 'Sequence cleared');
        }

        function updateSavedSequences() {
            const container = document.getElementById('saved-sequences');
            container.innerHTML = '';

            if (Object.keys(savedSequences).length === 0) {
                container.innerHTML = '<em>No saved sequences</em>';
                return;
            }

            Object.keys(savedSequences).forEach(name => {
                const sequence = savedSequences[name];
                const item = document.createElement('div');
                item.innerHTML = `
                    <strong>${name}</strong> (${sequence.length} commands)
                    <button class="btn btn-sm btn-info" onclick="loadSequenceByName('${name}')">Load</button>
                    <button class="btn btn-sm btn-danger" onclick="deleteSequence('${name}')">Delete</button>
                `;
                container.appendChild(item);
            });
        }

        function loadSequenceByName(name) {
            document.getElementById('sequence-name').value = name;
            loadSequence();
        }

        function deleteSequence(name) {
            if (confirm(`Delete sequence "${name}"?`)) {
                delete savedSequences[name];
                localStorage.setItem('robotSequences', JSON.stringify(savedSequences));
                updateSavedSequences();
                showStatus('info', `Sequence "${name}" deleted`);
            }
        }

        // Waypoint Navigation
        function addWaypoint() {
            const x = parseFloat(document.getElementById('waypoint-x').value);
            const y = parseFloat(document.getElementById('waypoint-y').value);
            const z = parseFloat(document.getElementById('waypoint-z').value || 0);

            if (isNaN(x) || isNaN(y)) {
                showStatus('error', 'Please enter valid coordinates');
                return;
            }

            waypoints.push({ x, y, z, id: waypoints.length + 1 });
            updateWaypointDisplay();
            updateMapVisualization();

            // Clear inputs
            document.getElementById('waypoint-x').value = '';
            document.getElementById('waypoint-y').value = '';
            document.getElementById('waypoint-z').value = '0';

            showStatus('success', `Waypoint ${waypoints.length} added`);
        }

        function updateWaypointDisplay() {
            const container = document.getElementById('waypoint-list');
            container.innerHTML = '';

            if (waypoints.length === 0) {
                container.innerHTML = '<em>No waypoints set</em>';
                return;
            }

            waypoints.forEach((wp, index) => {
                const item = document.createElement('div');
                item.innerHTML = `WP${wp.id}: E=${wp.x.toFixed(1)}, N=${wp.y.toFixed(1)}, U=${wp.z.toFixed(1)}`;
                container.appendChild(item);
            });
        }

        async function navigateWaypoints() {
            if (waypoints.length < 2) {
                showStatus('error', 'Need at least 2 waypoints for navigation');
                return;
            }

            showStatus('info', 'Starting waypoint navigation...');

            // Simple waypoint navigation - move between points
            // In a real implementation, this would use proper path planning
            for (let i = 1; i < waypoints.length; i++) {
                const current = waypoints[i - 1];
                const next = waypoints[i];

                // Calculate simple movement (simplified - would need proper navigation)
                const dx = next.x - current.x;
                const dy = next.y - current.y;

                // Determine primary direction
                let primaryCommand = 'f'; // Default forward
                if (Math.abs(dx) > Math.abs(dy)) {
                    primaryCommand = dx > 0 ? 'r' : 'l'; // Right or left
                } else {
                    primaryCommand = dy > 0 ? 'f' : 'b'; // Forward or backward
                }

                // Execute movement (simplified)
                await apiCall('/api/serial/send', { command: primaryCommand });
                await new Promise(resolve => setTimeout(resolve, 2000)); // Move for 2 seconds
                await apiCall('/api/serial/send', { command: 's' }); // Stop

                showStatus('info', `Reached waypoint ${i}`);
                await new Promise(resolve => setTimeout(resolve, 1000)); // Pause at waypoint
            }

            showStatus('success', 'Waypoint navigation completed');
        }

        function clearWaypoints() {
            waypoints = [];
            updateWaypointDisplay();
            updateMapVisualization();
            showStatus('info', 'Waypoints cleared');
        }

        // Waypoint Examples
        function loadWaypointExample(example) {
            const examples = {
                'square': [
                    { x: 0, y: 0, z: 0 },
                    { x: 4, y: 0, z: 0 },
                    { x: 4, y: 4, z: 0 },
                    { x: 0, y: 4, z: 0 }
                ],
                'triangle': [
                    { x: 0, y: 0, z: 0 },
                    { x: 3, y: 0, z: 0 },
                    { x: 1.5, y: 2.6, z: 0 }  // Equilateral triangle
                ],
                'circle': [
                    { x: 5, y: 0, z: 0 },
                    { x: 3.5, y: 3.5, z: 0 },
                    { x: 0, y: 5, z: 0 },
                    { x: -3.5, y: 3.5, z: 0 },
                    { x: -5, y: 0, z: 0 },
                    { x: -3.5, y: -3.5, z: 0 },
                    { x: 0, y: -5, z: 0 },
                    { x: 3.5, y: -3.5, z: 0 }
                ],
                'figure8': [
                    { x: 0, y: 0, z: 0 },
                    { x: 2, y: 2, z: 0 },
                    { x: 4, y: 0, z: 0 },
                    { x: 2, y: -2, z: 0 },
                    { x: 0, y: 0, z: 0 },
                    { x: -2, y: 2, z: 0 },
                    { x: -4, y: 0, z: 0 },
                    { x: -2, y: -2, z: 0 }
                ],
                'inspection': [
                    { x: 0, y: 0, z: 0 },
                    { x: 2, y: 0, z: 0 },
                    { x: 2, y: 3, z: 0 },
                    { x: 0, y: 3, z: 0 },
                    { x: 0, y: 6, z: 0 },
                    { x: 2, y: 6, z: 0 },
                    { x: 4, y: 6, z: 0 },
                    { x: 4, y: 3, z: 0 },
                    { x: 4, y: 0, z: 0 }
                ],
                'delivery': [
                    { x: 0, y: 0, z: 0 },      // Start/Home
                    { x: 1, y: 2, z: 0 },      // Stop 1
                    { x: 3, y: 1, z: 0 },      // Stop 2
                    { x: 2, y: 4, z: 0 },      // Stop 3
                    { x: 4, y: 3, z: 0 },      // Stop 4
                    { x: 0, y: 0, z: 0 }       // Return Home
                ]
            };

            if (examples[example]) {
            waypoints = examples[example].map((wp, index) => ({
                x: wp.x,
                y: wp.y,
                z: wp.z,
                id: index + 1
            }));
            updateWaypointDisplay();
            updateMapVisualization();
            showStatus('success', `${example.charAt(0).toUpperCase() + example.slice(1)} waypoint example loaded (${waypoints.length} waypoints)`);

                // Update example info
                const descriptions = {
                    'square': '4m × 4m square pattern - perfect for boundary testing',
                    'triangle': 'Equilateral triangle - good for corner navigation',
                    'circle': '8-point circle approximation - smooth curved path',
                    'figure8': 'Figure-8 pattern - tests complex path following',
                    'inspection': 'Systematic room coverage - like lawn mower for indoor spaces',
                    'delivery': 'Multi-stop delivery route with return to origin'
                };

                updateExampleInfo(descriptions[example] || 'Example loaded successfully');
            }
        }

        function updateExampleInfo(info) {
            const infoDiv = document.getElementById('example-info');
            infoDiv.innerHTML = `<strong>Current Example:</strong> ${info}<br><br>` +
                `<strong>Square Path:</strong> 4 waypoints forming a 4m × 4m square<br>` +
                `<strong>Triangle Path:</strong> 3 waypoints forming an equilateral triangle<br>` +
                `<strong>Circle Path:</strong> 8 waypoints approximating a 5m radius circle<br>` +
                `<strong>Figure-8:</strong> 8 waypoints creating a figure-8 pattern<br>` +
                `<strong>Room Inspection:</strong> Systematic room coverage pattern<br>` +
                `<strong>Delivery Route:</strong> Optimized multi-stop delivery path<br><br>` +
                `<em>Tip: Use ENU coordinates where East is X, North is Y, Up is Z</em>`;
        }

        // Current Position Management
        function setCurrentAsWaypoint() {
            const x = parseFloat(document.getElementById('current-x').textContent) || 0;
            const y = parseFloat(document.getElementById('current-y').textContent) || 0;
            const z = parseFloat(document.getElementById('current-z').textContent) || 0;

            waypoints.push({ x, y, z, id: waypoints.length + 1 });
            updateWaypointDisplay();
            showStatus('success', `Current position added as waypoint ${waypoints.length}`);
        }

        function resetOrigin() {
            document.getElementById('current-x').textContent = '0.0';
            document.getElementById('current-y').textContent = '0.0';
            document.getElementById('current-z').textContent = '0.0';
            document.getElementById('current-heading').textContent = '0.0';
            showStatus('info', 'Origin reset to (0,0,0)');
        }

        async function updateCurrentPosition() {
            try {
                const response = await fetch('/api/robot/position');
                const data = await response.json();

                if (data.success) {
                    document.getElementById('current-x').textContent = data.position.x.toFixed(2);
                    document.getElementById('current-y').textContent = data.position.y.toFixed(2);
                    document.getElementById('current-z').textContent = data.position.z.toFixed(2);
                    document.getElementById('current-heading').textContent = data.orientation.yaw.toFixed(1);

                    if (data.initialized) {
                        showStatus('success', `Position updated: (${data.position.x.toFixed(2)}, ${data.position.y.toFixed(2)})m`);
                    } else {
                        showStatus('warning', 'IMU position not yet initialized');
                    }
                } else {
                    showStatus('error', 'Failed to get current position: ' + data.error);
                }
            } catch (error) {
                showStatus('error', 'Network error getting position');
            }
        }

        // Waypoint Route Management
        function reverseWaypoints() {
            waypoints.reverse();
            // Reassign IDs
            waypoints.forEach((wp, index) => wp.id = index + 1);
            updateWaypointDisplay();
            showStatus('info', 'Waypoint route reversed');
        }

        function optimizeWaypoints() {
            if (waypoints.length < 3) {
                showStatus('warning', 'Need at least 3 waypoints to optimize');
                return;
            }

            // Simple nearest neighbor optimization (could be enhanced with TSP)
            const optimized = [waypoints[0]]; // Start with first waypoint
            const remaining = waypoints.slice(1);

            while (remaining.length > 0) {
                const last = optimized[optimized.length - 1];
                let nearestIndex = 0;
                let nearestDistance = Infinity;

                remaining.forEach((wp, index) => {
                    const distance = Math.sqrt(
                        Math.pow(wp.x - last.x, 2) +
                        Math.pow(wp.y - last.y, 2) +
                        Math.pow(wp.z - last.z, 2)
                    );
                    if (distance < nearestDistance) {
                        nearestDistance = distance;
                        nearestIndex = index;
                    }
                });

                optimized.push(remaining[nearestIndex]);
                remaining.splice(nearestIndex, 1);
            }

            waypoints = optimized.map((wp, index) => ({ ...wp, id: index + 1 }));
            updateWaypointDisplay();
            showStatus('success', 'Route optimized using nearest neighbor algorithm');
        }

        function saveWaypointRoute() {
            const name = prompt('Enter route name:');
            if (!name) return;

            const routes = JSON.parse(localStorage.getItem('waypointRoutes') || '{}');
            routes[name] = [...waypoints];
            localStorage.setItem('waypointRoutes', JSON.stringify(routes));

            updateSavedRoutes();
            showStatus('success', `Route "${name}" saved`);
        }

        function loadWaypointRoute() {
            const routes = JSON.parse(localStorage.getItem('waypointRoutes') || '{}');
            const routeNames = Object.keys(routes);

            if (routeNames.length === 0) {
                showStatus('warning', 'No saved routes found');
                return;
            }

            const name = prompt(`Available routes: ${routeNames.join(', ')}\nEnter route name:`);
            if (!name || !routes[name]) {
                showStatus('error', 'Route not found');
                return;
            }

            waypoints = routes[name].map((wp, index) => ({ ...wp, id: index + 1 }));
            updateWaypointDisplay();
            updateMapVisualization();
            showStatus('success', `Route "${name}" loaded (${waypoints.length} waypoints)`);
        }

        function updateSavedRoutes() {
            // This could display saved routes in a list if needed
            console.log('Saved routes updated');
        }

        // Simple Map Rendering Functions
        function initializeMapCanvas() {
            if (!mapCanvas) {
                mapCanvas = document.getElementById('waypoint-canvas');
                if (mapCanvas) {
                    mapContext = mapCanvas.getContext('2d');
                    updateMapVisualization();
                }
            }
        }

        function updateMapVisualization() {
            initializeMapCanvas();
            if (!mapContext) return;

            const canvas = mapCanvas;
            const ctx = mapContext;

            // Clear canvas
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // Draw simple grid
            ctx.strokeStyle = '#374151';
            ctx.lineWidth = 1;
            const gridSize = 50;

            for (let x = 0; x <= canvas.width; x += gridSize) {
                ctx.beginPath();
                ctx.moveTo(x, 0);
                ctx.lineTo(x, canvas.height);
                ctx.stroke();
            }

            for (let y = 0; y <= canvas.height; y += gridSize) {
                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.lineTo(canvas.width, y);
                ctx.stroke();
            }

            // Draw origin
            const centerX = canvas.width / 2;
            const centerY = canvas.height / 2;
            ctx.strokeStyle = '#10b981';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(centerX - 10, centerY);
            ctx.lineTo(centerX + 10, centerY);
            ctx.moveTo(centerX, centerY - 10);
            ctx.lineTo(centerX, centerY + 10);
            ctx.stroke();

            // Draw waypoints
            waypoints.forEach((wp, index) => {
                const wpX = centerX + (wp.x * mapScale);
                const wpY = centerY - (wp.y * mapScale);

                ctx.fillStyle = '#3b82f6';
                ctx.beginPath();
                ctx.arc(wpX, wpY, 6, 0, 2 * Math.PI);
                ctx.fill();

                ctx.fillStyle = '#ffffff';
                ctx.font = '10px Arial';
                ctx.textAlign = 'center';
                ctx.fillText((index + 1).toString(), wpX, wpY + 3);
            });
        }

        function clearMapCanvas() {
            if (mapContext && mapCanvas) {
                mapContext.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
            }
        }

        async function loadMap() {
            try {
                const response = await fetch('/api/map/canvas');
                const html = await response.text();
                document.getElementById('map-container').innerHTML = html;
            } catch (error) {
                console.error('Failed to load map:', error);
                document.getElementById('map-container').innerHTML = '<p style="color: #ef4444;">Failed to load map</p>';
            }
        }

        // Update dashboard position display
        async function updateDashboardPosition() {
            try {
                const response = await fetch('/api/robot/position');
                const data = await response.json();

                if (data.success && data.initialized) {
                    document.getElementById('pos-x').textContent = data.position.x.toFixed(2) + 'm';
                    document.getElementById('pos-y').textContent = data.position.y.toFixed(2) + 'm';
                    document.getElementById('pos-z').textContent = data.position.z.toFixed(2) + 'm';
                    document.getElementById('pos-heading').textContent = data.orientation.yaw.toFixed(1) + '°';
                } else {
                    document.getElementById('pos-x').textContent = '--';
                    document.getElementById('pos-y').textContent = '--';
                    document.getElementById('pos-z').textContent = '--';
                    document.getElementById('pos-heading').textContent = '--°';
                }
            } catch (error) {
                document.getElementById('pos-x').textContent = '--';
                document.getElementById('pos-y').textContent = '--';
                document.getElementById('pos-z').textContent = '--';
                document.getElementById('pos-heading').textContent = '--°';
            }
        }

        // Mega connection management
        async function updateMegaStatus() {
            try {
                const response = await fetch('/api/mega/status');
                const data = await response.json();

                if (data.success) {
                    const status = data.status;
                    const dot = document.getElementById('mega-connection-dot');
                    const statusText = document.getElementById('mega-status');
                    const portText = document.getElementById('mega-port');
                    const reconnectBtn = document.getElementById('mega-reconnect-btn');

                    // Update connection status
                    if (status.connected) {
                        dot.style.backgroundColor = '#10b981'; // Green
                        statusText.textContent = 'Connected';
                        statusText.style.color = '#10b981';
                        portText.textContent = status.port || '--';
                        reconnectBtn.style.display = 'none';
                    } else {
                        dot.style.backgroundColor = '#ef4444'; // Red
                        statusText.textContent = 'Disconnected';
                        statusText.style.color = '#ef4444';
                        portText.textContent = '--';
                        reconnectBtn.style.display = 'inline-block';
                    }

                    // Show reconnection attempts if applicable
                    if (!status.connected && status.reconnect_attempts > 0) {
                        statusText.textContent = `Reconnecting (${status.reconnect_attempts})`;
                    }
                }
            } catch (error) {
                console.error('Failed to update Mega status:', error);
                // Fallback status
                document.getElementById('mega-connection-dot').style.backgroundColor = '#f59e0b'; // Yellow
                document.getElementById('mega-status').textContent = 'Status Check Failed';
                document.getElementById('mega-status').style.color = '#f59e0b';
                document.getElementById('mega-port').textContent = '--';
            }
        }

        async function reconnectMega() {
            const reconnectBtn = document.getElementById('mega-reconnect-btn');
            const originalText = reconnectBtn.innerHTML;

            // Show loading state
            reconnectBtn.innerHTML = '⟳';
            reconnectBtn.disabled = true;

            try {
                const response = await fetch('/api/mega/reconnect', { method: 'POST' });
                const data = await response.json();

                if (data.success) {
                    showStatus('success', 'Mega reconnection initiated');
                    // Update status after a short delay
                    setTimeout(updateMegaStatus, 2000);
                } else {
                    showStatus('error', 'Mega reconnection failed: ' + (data.error || 'Unknown error'));
                }
            } catch (error) {
                showStatus('error', 'Failed to initiate reconnection: ' + error.message);
            } finally {
                // Restore button
                reconnectBtn.innerHTML = originalText;
                reconnectBtn.disabled = false;
            }
        }

        // Serial Monitor Functions
        async function startSerialMonitor() {
            if (serialMonitorActive) return;

            serialMonitorActive = true;
            serialMonitorStartTime = Date.now();
            serialStats = { totalBytes: 0, linesReceived: 0, errorsDetected: 0, lastDataRate: 0 };

            // Update UI
            const startBtn = document.getElementById('monitor-start-btn');
            const stopBtn = document.getElementById('monitor-stop-btn');
            const statusIndicator = document.getElementById('monitor-connection-status');
            const statusText = document.getElementById('monitor-connection-text');

            if (startBtn && stopBtn && statusIndicator && statusText) {
                startBtn.style.display = 'none';
                stopBtn.style.display = 'flex';
                statusIndicator.style.backgroundColor = '#10b981';
                statusText.textContent = 'Monitoring';

                // Update status text
                const statusTextEl = document.getElementById('serial-status-text');
                if (statusTextEl) {
                    statusTextEl.textContent = 'Serial monitor active - receiving data...';
                }

                // Update debug info
                const debugInfo = document.getElementById('debug-info');
                if (debugInfo) {
                    debugInfo.textContent = 'Monitor started at ' + new Date().toLocaleTimeString();
                }
            }

            // Clear previous data
            clearSerialMonitor();

            // Add initial message
            addToSerialMonitor('[INFO] Serial monitor started', 'info');

            // Start monitoring loop
            serialMonitorInterval = setInterval(fetchSerialData, 1000); // Poll every second

            showStatus('success', 'Serial monitor started');
        }

        async         function stopSerialMonitor() {
            console.log('Stopping Serial Monitor...');
            if (!serialMonitorActive) return;

            serialMonitorActive = false;
            clearInterval(serialMonitorInterval);

            // Update UI
            const startBtn = document.getElementById('monitor-start-btn');
            const stopBtn = document.getElementById('monitor-stop-btn');
            const statusIndicator = document.getElementById('monitor-connection-status');
            const statusText = document.getElementById('monitor-connection-text');

            if (startBtn && stopBtn && statusIndicator && statusText) {
                startBtn.style.display = 'flex';
                stopBtn.style.display = 'none';
                statusIndicator.style.backgroundColor = 'var(--text-muted)';
                statusText.textContent = 'Disconnected';

                // Reset status text
                const statusTextEl = document.getElementById('serial-status-text');
                if (statusTextEl) {
                    statusTextEl.textContent = 'Serial monitor stopped. Click "Start Monitor" to begin receiving data.';
                }

                // Update debug info
                const debugInfo = document.getElementById('debug-info');
                if (debugInfo) {
                    debugInfo.textContent = 'Monitor stopped at ' + new Date().toLocaleTimeString();
                }
            }

            // Add final message
            addToSerialMonitor('[INFO] Serial monitor stopped', 'info');
            updateMonitorStats();

            showStatus('info', 'Serial monitor stopped');
        }

        // Test function to manually add data to Serial Monitor
        function testSerialMonitor() {
            console.log('Testing Serial Monitor display...');

            // First, try direct DOM manipulation to verify element exists
            const output = document.getElementById('serial-monitor-output');
            console.log('Direct DOM test - output element:', output);
            if (output) {
                console.log('Output element innerHTML before:', output.innerHTML.substring(0, 100) + '...');
            }

            // Test with simple text addition
            addToSerialMonitor('[TEST] Serial Monitor test message 1', 'data');
            addToSerialMonitor('[TEST] Serial Monitor test message 2', 'info');
            addToSerialMonitor('[TEST] Serial Monitor test message 3', 'warning');
            addToSerialMonitor('[TEST] Serial Monitor test message 4', 'error');

            // Update debug info
            const debugInfo = document.getElementById('debug-info');
            if (debugInfo) {
                debugInfo.textContent = 'Test data added at ' + new Date().toLocaleTimeString();
            }

            // Check final state
            if (output) {
                console.log('Output element innerHTML after:', output.innerHTML.substring(0, 200) + '...');
                console.log('Output has', output.children.length, 'children');
            }

            showStatus('success', 'Test data added to Serial Monitor');
        }

        // Force show data by directly injecting HTML
        function forceShowData() {
            console.log('Force showing data...');
            const output = document.getElementById('serial-monitor-output');
            if (!output) {
                console.error('Cannot find serial-monitor-output element!');
                alert('Cannot find output element!');
                return;
            }

            // Hide placeholder
            const placeholder = document.getElementById('serial-monitor-placeholder');
            if (placeholder) {
                placeholder.style.display = 'none';
            }

            // Clear and add test content directly with HIGH CONTRAST
            output.innerHTML = `
                <div style="color: #ffffff !important; background: #374151 !important; padding: 4px 8px !important; margin: 2px 0 !important; border-radius: 4px !important; border-bottom: 1px solid #333 !important; font-family: 'JetBrains Mono', monospace !important; font-size: 13px !important;">
                    <span style="color: #cccccc !important; font-size: 11px !important;">[${new Date().toLocaleTimeString()}]</span>
                    <span>[FORCE] This is test data injected directly - HIGH CONTRAST</span>
                </div>
                <div style="color: #ffffff !important; background: #2563eb !important; padding: 4px 8px !important; margin: 2px 0 !important; border-radius: 4px !important; border-bottom: 1px solid #333 !important; font-family: 'JetBrains Mono', monospace !important; font-size: 13px !important;">
                    <span style="color: #cccccc !important; font-size: 11px !important;">[${new Date().toLocaleTimeString()}]</span>
                    <span>[FORCE] Serial Monitor is working with visible colors!</span>
                </div>
                <div style="color: #000000 !important; background: #f59e0b !important; padding: 4px 8px !important; margin: 2px 0 !important; border-radius: 4px !important; border-bottom: 1px solid #333 !important; font-family: 'JetBrains Mono', monospace !important; font-size: 13px !important;">
                    <span style="color: #333333 !important; font-size: 11px !important;">[${new Date().toLocaleTimeString()}]</span>
                    <span>[FORCE] Test message 3 - Orange background with black text</span>
                </div>
            `;

            console.log('Force injected HTML content');
            showStatus('success', 'Data force-injected into Serial Monitor');
        }

        function clearSerialMonitor() {
            console.log('Clearing Serial Monitor...');
            const output = document.getElementById('serial-monitor-output');
            if (output) {
                output.innerHTML = '<div id="serial-monitor-placeholder" style="text-align: center; color: var(--text-muted); padding: 40px;"><div style="font-size: 48px; margin-bottom: 15px;">📟</div><div id="serial-status-text">Serial monitor cleared. Click "Start Monitor" to begin receiving data.</div><div style="margin-top: 10px; font-size: 14px; opacity: 0.7;">Raw data from Arduino Mega will appear here in real-time.</div><div id="debug-info" style="margin-top: 20px; font-size: 12px; color: var(--accent-blue);"></div></div>';
            }
            serialStats.totalBytes = 0;
            serialStats.linesReceived = 0;
            serialStats.errorsDetected = 0;
            statusUpdateCount = 0;
            updateMonitorStats();

            // Reset status badge
            const statusBadge = document.getElementById('data-rate-badge');
            if (statusBadge) {
                statusBadge.textContent = '0 B/s';
            }
        }

        async function fetchSerialData() {
            if (!serialMonitorActive) return;

            try {
                const response = await fetch('/api/serial/monitor');
                const data = await response.json();

                if (data.success && data.data && data.data.length > 0) {
                    // Process each line of serial data
                    data.data.forEach(line => {
                        addToSerialMonitor(line, 'data');
                        serialStats.totalBytes += line.length;
                        serialStats.linesReceived++;

                        // Check for errors
                        if (line.toLowerCase().includes('error') ||
                            line.toLowerCase().includes('fail') ||
                            line.includes('!')) {
                            serialStats.errorsDetected++;
                        }
                    });

                    updateMonitorStats();
                    updateDataRate();

                    // Update status indicator
                    statusUpdateCount++;
                    const statusBadge = document.getElementById('data-rate-badge');
                    if (statusBadge) {
                        statusBadge.textContent = `${statusUpdateCount} updates`;
                    }
                } else {
                    // No data received - show this
                    addToSerialMonitor('[INFO] No new serial data available', 'info');
                }
            } catch (error) {
                console.error('Serial monitor fetch error:', error);
                // Silent fail - Mega might not be connected
                if (serialMonitorActive) {
                    addToSerialMonitor('[WARNING] Failed to fetch serial data: ' + error.message, 'warning');
                }
            }
        }

        function addToSerialMonitor(text, type = 'data') {
            console.log('addToSerialMonitor called with:', text.substring(0, 50) + '...', 'type:', type);
            const output = document.getElementById('serial-monitor-output');
            console.log('Found output element:', !!output, 'element:', output);
            if (!output) {
                console.error('serial-monitor-output element not found!');
                return;
            }

            // Force scroll to bottom for new messages
            const shouldScroll = output.scrollTop + output.clientHeight >= output.scrollHeight - 10;

            const autoscroll = document.getElementById('monitor-autoscroll').checked;
            const showTimestamps = document.getElementById('monitor-timestamps').checked;

            // Create message element with HIGH CONTRAST styling
            const messageDiv = document.createElement('div');
            messageDiv.style.cssText = `
                margin-bottom: 2px !important;
                font-family: 'JetBrains Mono', 'Fira Code', 'Courier New', monospace !important;
                font-size: 13px !important;
                line-height: 1.4 !important;
                padding: 4px 8px !important;
                border-radius: 4px !important;
                border-bottom: 1px solid #333 !important;
                white-space: pre-wrap !important;
                word-break: break-all !important;
                display: block !important;
                visibility: visible !important;
                opacity: 1 !important;
                min-height: 20px !important;
            `;

            let timestamp = '';
            if (showTimestamps) {
                const now = new Date();
                timestamp = `[${now.toLocaleTimeString()}] `;
            }

            // HIGH CONTRAST color coding with background colors
            let textColor = '#ffffff'; // White text
            let bgColor = 'transparent';

            switch (type) {
                case 'error':
                    textColor = '#ffffff';
                    bgColor = '#dc2626'; // Red background
                    break;
                case 'warning':
                    textColor = '#000000';
                    bgColor = '#f59e0b'; // Orange background
                    break;
                case 'info':
                    textColor = '#ffffff';
                    bgColor = '#2563eb'; // Blue background
                    break;
                case 'success':
                    textColor = '#ffffff';
                    bgColor = '#16a34a'; // Green background
                    break;
                default:
                    textColor = '#ffffff';
                    bgColor = '#374151'; // Gray background
            }

            messageDiv.style.color = textColor + ' !important';
            messageDiv.style.backgroundColor = bgColor + ' !important';
            messageDiv.textContent = timestamp + text;

            // Hide placeholder if it exists
            const placeholder = document.getElementById('serial-monitor-placeholder');
            if (placeholder) {
                console.log('Hiding placeholder');
                placeholder.style.display = 'none';
            }

            // Clear initial content if this is the first real message
            if (output.children.length === 1 && output.firstChild.id === 'serial-monitor-placeholder') {
                console.log('Clearing placeholder content');
                output.innerHTML = '';
            }

            console.log('Appending message div to output');
            output.appendChild(messageDiv);

            // Auto-scroll to bottom if user was already at bottom
            if (shouldScroll) {
                output.scrollTop = output.scrollHeight;
            }

            console.log('Message added successfully, output now has', output.children.length, 'children');

            // Auto-scroll if enabled
            if (autoscroll) {
                output.scrollTop = output.scrollHeight;
            }

            // Limit lines to prevent memory issues (keep last 1000 lines)
            while (output.children.length > 1000) {
                output.removeChild(output.firstChild);
            }
        }

        function updateMonitorStats() {
            document.getElementById('stats-total-bytes').textContent = serialStats.totalBytes;
            document.getElementById('stats-lines').textContent = serialStats.linesReceived;
            document.getElementById('stats-errors').textContent = serialStats.errorsDetected;

            if (serialMonitorStartTime) {
                const elapsed = Math.floor((Date.now() - serialMonitorStartTime) / 1000);
                const hours = Math.floor(elapsed / 3600);
                const minutes = Math.floor((elapsed % 3600) / 60);
                const seconds = elapsed % 60;
                document.getElementById('stats-runtime').textContent =
                    `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
            }
        }

        function updateDataRate() {
            const elapsed = (Date.now() - serialMonitorStartTime) / 1000;
            if (elapsed > 0) {
                const rate = Math.round(serialStats.totalBytes / elapsed);
                document.getElementById('data-rate-badge').textContent = `${rate} B/s`;
                serialStats.lastDataRate = rate;
            }
        }

        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            updateSpeedDisplay();
            updateWheelSpeedDisplay();
            updateGripperTiltDisplay();
            updateSequenceDisplay();
            updateSavedSequences();
            updateWaypointDisplay();

            // Load map when Path Planning tab is shown
            const pathPlanningTab = document.getElementById('pathplanning');
            if (pathPlanningTab && !pathPlanningTab.classList.contains('hidden')) {
                loadMap();
            }

            // Initial status updates
            updateMegaStatus();

            // Auto-refresh sensors every 5 seconds
            setInterval(refreshSensors, 5000);

            // Update robot position on map and dashboard every 2 seconds
            // setInterval(updateRobotPositionOnMap, 2000); // TODO: Define this function
            setInterval(updateDashboardPosition, 2000);

            // Update Mega status every 10 seconds
            setInterval(updateMegaStatus, 10000);

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

    def __init__(self, mega_interface=None, sensor_manager=None, ros2_interface=None, simulation_mode=False, main_app=None):
        logger.info("Initializing Flask application...")
        self.app = Flask(__name__)
        self.mega = mega_interface
        self.sensors = sensor_manager
        self.ros2 = ros2_interface
        self.simulation_mode = simulation_mode
        self.main_app = main_app  # Reference to main app for IMU access

        logger.info(f"Mega interface: {type(mega_interface)}")
        logger.info(f"Sensor manager: {type(sensor_manager)}")
        logger.info(f"ROS2 interface: {type(ros2_interface)}")

        # Setup routes
        try:
            self._setup_routes()
            logger.info("Routes setup completed")
        except Exception as e:
            logger.error(f"Failed to setup routes: {e}")
            raise

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

        # Path Planning
        @self.app.route('/api/robot/sequences/execute', methods=['POST'])
        def execute_sequence():
            return self._execute_sequence()

        @self.app.route('/api/robot/sequences/save', methods=['POST'])
        def save_sequence():
            return self._save_sequence()

        @self.app.route('/api/robot/sequences/load/<name>', methods=['GET'])
        def load_sequence(name):
            return self._load_sequence(name)

        @self.app.route('/api/robot/sequences/list', methods=['GET'])
        def list_sequences():
            return self._list_sequences()

        @self.app.route('/api/robot/waypoints/navigate', methods=['POST'])
        def navigate_waypoints():
            return self._navigate_waypoints()

        @self.app.route('/api/robot/position', methods=['GET'])
        def get_current_position():
            return self._get_current_position()

        @self.app.route('/api/map/canvas', methods=['GET'])
        def get_map_canvas():
            return self._get_map_canvas(), 200, {'Content-Type': 'text/html'}

        @self.app.route('/api/serial/monitor', methods=['GET'])
        def get_serial_monitor_data():
            return self._get_serial_monitor_data()

        @self.app.route('/api/mega/status', methods=['GET'])
        def get_mega_status():
            return self._get_mega_status()

        @self.app.route('/api/mega/reconnect', methods=['POST'])
        def reconnect_mega():
            return self._reconnect_mega()

        # End of route setup
        pass

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

            # Validate command - complete set from COMMANDS.md
            valid_commands = [
                # Movement commands (all cardinal, diagonal, rotation, turning)
                'f', 'b', 'l', 'r', 'q', 'e', 'z', 'x', 'c', 'w', 't', 'y', 'a', 'j', 's',
                # Control commands
                'p', 'v', 'o',
                # Speed control (50%-100%)
                '5', '6', '7', '8', '9', '0',
                # Lifter commands
                'u', 'd',
                # Testing commands
                '1', '2', '3', '4', 'g', 'h',
                # Servo commands
                'mu', 'md', 'mc', 'no', 'nc', 'nh',
                # Sensor commands
                'sr', 'ls',
                # Safety commands
                'se', 'sd',
                # Publishing commands
                'spe', 'spd'
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

    def _execute_sequence(self):
        """Execute a movement sequence"""
        try:
            data = request.get_json()
            if not data or 'sequence' not in data:
                return jsonify({'success': False, 'error': 'Sequence required', 'timestamp': time.time()}), 400

            sequence = data['sequence']
            if not isinstance(sequence, list) or len(sequence) == 0:
                return jsonify({'success': False, 'error': 'Valid sequence list required', 'timestamp': time.time()}), 400

            # Execute sequence in background
            threading.Thread(target=self._execute_sequence_async, args=(sequence,), daemon=True).start()

            return jsonify({
                'success': True,
                'message': f'Sequence execution started ({len(sequence)} commands)',
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Sequence execution error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _execute_sequence_async(self, sequence):
        """Execute sequence asynchronously"""
        try:
            for command in sequence:
                if self.mega:
                    self.mega.send_command_to_mega(command)
                # Add delay between commands
                time.sleep(0.5)

            logger.info(f'Sequence execution completed: {len(sequence)} commands')

        except Exception as e:
            logger.error(f'Async sequence execution error: {str(e)}')

    def _save_sequence(self):
        """Save a movement sequence"""
        try:
            data = request.get_json()
            if not data or 'name' not in data or 'sequence' not in data:
                return jsonify({'success': False, 'error': 'Name and sequence required', 'timestamp': time.time()}), 400

            name = data['name']
            sequence = data['sequence']

            if not isinstance(sequence, list):
                return jsonify({'success': False, 'error': 'Sequence must be a list', 'timestamp': time.time()}), 400

            # In a real implementation, this would save to a database
            # For now, we'll just acknowledge the save
            logger.info(f'Sequence saved: {name} ({len(sequence)} commands)')

            return jsonify({
                'success': True,
                'message': f'Sequence "{name}" saved ({len(sequence)} commands)',
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Save sequence error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _load_sequence(self, name):
        """Load a movement sequence"""
        try:
            # In a real implementation, this would load from a database
            # For now, return a placeholder
            return jsonify({
                'success': True,
                'sequence': ['f', 's'],  # Placeholder sequence
                'name': name,
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Load sequence error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _list_sequences(self):
        """List saved sequences"""
        try:
            # In a real implementation, this would query a database
            # For now, return placeholder data
            sequences = {
                'lawn_mower': {'commands': 10, 'description': 'Systematic coverage pattern'},
                'spiral_search': {'commands': 8, 'description': 'Expanding spiral search'},
                'boundary_follow': {'commands': 12, 'description': 'Follow perimeter boundary'}
            }

            return jsonify({
                'success': True,
                'sequences': sequences,
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'List sequences error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _navigate_waypoints(self):
        """Navigate through waypoints"""
        try:
            data = request.get_json()
            if not data or 'waypoints' not in data:
                return jsonify({'success': False, 'error': 'Waypoints required', 'timestamp': time.time()}), 400

            waypoints = data['waypoints']
            if not isinstance(waypoints, list) or len(waypoints) < 2:
                return jsonify({'success': False, 'error': 'At least 2 waypoints required', 'timestamp': time.time()}), 400

            # Execute waypoint navigation in background
            threading.Thread(target=self._navigate_waypoints_async, args=(waypoints,), daemon=True).start()

            return jsonify({
                'success': True,
                'message': f'Waypoint navigation started ({len(waypoints)} waypoints)',
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Waypoint navigation error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _navigate_waypoints_async(self, waypoints):
        """Navigate waypoints asynchronously using IMU-based dead reckoning"""
        try:
            logger.info(f'Starting IMU-based waypoint navigation with {len(waypoints)} waypoints')

            # Access the main app's IMU position tracking
            main_app = None
            if hasattr(self, 'main_app'):
                main_app = self.main_app

            for i, waypoint in enumerate(waypoints):
                if i == 0:
                    logger.info('Starting from first waypoint (origin)')
                    continue  # Skip first waypoint (starting point)

                target_x, target_y, target_z = waypoint
                logger.info(f'Navigating to waypoint {i + 1}: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})')

                # Navigate to waypoint using IMU-based position tracking
                success = self._navigate_to_coordinate_imu(target_x, target_y, main_app)

                if success:
                    logger.info(f'Successfully reached waypoint {i + 1}')
                else:
                    logger.warning(f'Failed to reach waypoint {i + 1} within timeout, continuing to next waypoint')

                # Brief pause at waypoint
                time.sleep(2)

            logger.info(f'IMU-based waypoint navigation completed: {len(waypoints)} waypoints')

        except Exception as e:
            logger.error(f'IMU waypoint navigation error: {str(e)}')

    def _navigate_to_coordinate_imu(self, target_x, target_y, main_app=None):
        """Navigate to specific coordinate using IMU dead reckoning"""
        try:
            timeout = 30.0  # 30 second timeout per waypoint
            start_time = time.time()
            tolerance = 0.2  # 20cm tolerance

            while time.time() - start_time < timeout:
                # Get current position from IMU tracking
                if main_app and hasattr(main_app, 'get_current_position'):
                    current_state = main_app.get_current_position()
                    if not current_state['initialized']:
                        logger.warning("IMU position not initialized, waiting...")
                        time.sleep(1)
                        continue

                    current_pos = current_state['position']
                    current_heading = current_state['orientation'][2]  # yaw
                else:
                    # Fallback to simple position tracking
                    current_pos = [0.0, 0.0, 0.0]
                    current_heading = 0.0

                # Calculate distance and bearing to target
                dx = target_x - current_pos[0]
                dy = target_y - current_pos[1]
                distance = math.sqrt(dx*dx + dy*dy)

                logger.debug(f'Current: ({current_pos[0]:.2f}, {current_pos[1]:.2f}) '
                           f'Heading: {current_heading:.1f}°, '
                           f'Target: ({target_x:.1f}, {target_y:.1f}), '
                           f'Distance: {distance:.2f}m')

                # Check if we've reached the waypoint
                if distance <= tolerance:
                    logger.info(f'Waypoint reached! Distance: {distance:.2f}m')
                    if self.mega:
                        self.mega.send_command_to_mega('s')  # Stop
                    return True

                # Calculate required bearing to target
                target_bearing = math.degrees(math.atan2(dy, dx))

                # Calculate turn angle
                turn_angle = target_bearing - current_heading

                # Normalize turn angle to -180 to 180 degrees
                while turn_angle > 180:
                    turn_angle -= 360
                while turn_angle < -180:
                    turn_angle += 360

                # Execute turn if needed (5 degree tolerance)
                if abs(turn_angle) > 5:
                    turn_command = 'e' if turn_angle > 0 else 'q'  # e=right turn, q=left turn
                    turn_time = min(abs(turn_angle) / 45.0, 2.0)  # Max 2 seconds turn

                    if self.mega:
                        logger.debug(f'Turning {turn_angle:.1f}° for {turn_time:.1f}s')
                        self.mega.send_command_to_mega(turn_command)
                        time.sleep(turn_time)
                        self.mega.send_command_to_mega('s')
                        time.sleep(0.5)  # Brief pause

                # Move forward (proportional to remaining distance)
                if distance > tolerance:
                    # Speed based on distance (slower when close)
                    speed_factor = min(distance / 2.0, 1.0)  # Max speed at 2m distance
                    move_time = min(distance / 0.3 * speed_factor, 5.0)  # Max 5 seconds

                    if self.mega:
                        logger.debug(f'Moving forward for {move_time:.1f}s (distance: {distance:.2f}m)')
                        self.mega.send_command_to_mega('f')
                        time.sleep(move_time)
                        self.mega.send_command_to_mega('s')
                        time.sleep(0.5)  # Brief pause

                time.sleep(0.2)  # Small delay between navigation iterations

            logger.warning(f'Waypoint navigation timeout after {timeout}s')
            return False

        except Exception as e:
            logger.error(f'IMU coordinate navigation error: {str(e)}')
            return False

    def _get_current_position(self):
        """Get current robot position from IMU tracking"""
        try:
            if self.main_app and hasattr(self.main_app, 'get_current_position'):
                position_data = self.main_app.get_current_position()
                return jsonify({
                    'success': True,
                    'position': {
                        'x': position_data['position'][0],
                        'y': position_data['position'][1],
                        'z': position_data['position'][2]
                    },
                    'orientation': {
                        'roll': position_data['orientation'][0],
                        'pitch': position_data['orientation'][1],
                        'yaw': position_data['orientation'][2]
                    },
                    'initialized': position_data['initialized'],
                    'timestamp': time.time()
                })
            else:
                return jsonify({
                    'success': False,
                    'error': 'IMU position tracking not available',
                    'timestamp': time.time()
                }), 503

        except Exception as e:
            logger.error(f'Get current position error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _get_map_canvas(self):
        """Return the map canvas HTML separately to avoid template truncation"""
        map_html = '''<canvas id="waypoint-canvas" width="600" height="400" style="border: 2px solid #374151; border-radius: 8px; background: #1f2937; width: 100%; max-width: 600px;"></canvas>'''
        return map_html

    def _get_serial_monitor_data(self):
        """Get raw serial data from Mega for monitoring with enhanced status"""
        try:
            data_lines = []

            if self.mega:
                # Use the enhanced read method
                data_lines = self.mega.read_available_data(max_lines=50)

                # Add connection status messages
                if not self.mega.mega_connected:
                    if len(data_lines) == 0:  # Only add if no data
                        data_lines.append('[WARNING] Arduino Mega not connected')
                        if self.mega.auto_reconnect:
                            reconnect_info = self.mega.get_connection_status()
                            attempts = reconnect_info.get('reconnect_attempts', 0)
                            max_attempts = getattr(self.mega, 'max_reconnect_attempts', 5)
                            data_lines.append(f'[INFO] Auto-reconnecting... ({attempts}/{max_attempts})')
            else:
                data_lines.append('[ERROR] Mega interface not initialized')

            # Get connection status
            connection_status = self.mega.get_connection_status() if self.mega else {}

            return jsonify({
                'success': True,
                'data': data_lines,
                'timestamp': time.time(),
                'connection': {
                    'connected': connection_status.get('connected', False),
                    'port': connection_status.get('port'),
                    'auto_reconnect': connection_status.get('auto_reconnect', False),
                    'reconnect_attempts': connection_status.get('reconnect_attempts', 0),
                    'last_activity': connection_status.get('last_activity', 0),
                    'healthy': self.mega.is_healthy() if self.mega else False
                },
                'stats': connection_status.get('stats', {})
            })

        except Exception as e:
            logger.error(f'Serial monitor data error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'data': [],
                'timestamp': time.time(),
                'connection': {
                    'connected': False,
                    'healthy': False
                },
                'stats': {}
            }), 500

    def _get_mega_status(self):
        """Get detailed Mega connection status"""
        try:
            if not self.mega:
                return jsonify({
                    'success': False,
                    'error': 'Mega interface not initialized',
                    'status': {}
                }), 503

            status = self.mega.get_connection_status()
            return jsonify({
                'success': True,
                'status': status,
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Mega status error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'status': {},
                'timestamp': time.time()
            }), 500

    def _reconnect_mega(self):
        """Force Mega reconnection"""
        try:
            if not self.mega:
                return jsonify({
                    'success': False,
                    'error': 'Mega interface not initialized'
                }), 503

            logger.info('Forcing Mega reconnection via API')
            success = self.mega.force_reconnect()

            return jsonify({
                'success': success,
                'message': 'Reconnection ' + ('successful' if success else 'failed'),
                'timestamp': time.time()
            })

        except Exception as e:
            logger.error(f'Mega reconnection error: {str(e)}')
            return jsonify({
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }), 500

    def _path_to_commands(self, path):
        """Convert path coordinates to movement commands"""
        commands = []

        for i in range(1, len(path)):
            current = path[i - 1]
            next_pos = path[i]

            # Calculate movement direction
            dx = next_pos[0] - current[0]
            dy = next_pos[1] - current[1]

            # Determine primary direction
            if abs(dx) > abs(dy):
                command = 'r' if dx > 0 else 'l'
            else:
                command = 'f' if dy > 0 else 'b'

            commands.append(command)

        commands.append('s')  # Stop at destination
        return commands

    def run(self, host=FLASK_HOST, port=FLASK_PORT, debug=FLASK_DEBUG):
        """Run the Flask application"""
        logger.info(f"Starting Flask web server on http://{host}:{port}")
        self.app.run(host=host, port=port, debug=debug)
