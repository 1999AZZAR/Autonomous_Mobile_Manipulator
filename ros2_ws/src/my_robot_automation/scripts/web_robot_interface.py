#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, render_template_string, jsonify, request
import threading
import time

# Modern Professional Web Interface Template
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Autonomous Mobile Manipulator Control Center</title>
    <link href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css" rel="stylesheet">
    <style>
        :root {
            --primary: #2563eb;
            --secondary: #64748b;
            --success: #10b981;
            --warning: #f59e0b;
            --danger: #ef4444;
            --dark: #1e293b;
            --light: #f8fafc;
            --border: #e2e8f0;
            --shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
            --shadow-lg: 0 10px 15px -3px rgba(0, 0, 0, 0.1);
        }

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            color: var(--dark);
        }

        .app-container {
            max-width: 1400px;
            margin: 0 auto;
            padding: 20px;
        }

        .header {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 16px;
            padding: 24px;
            margin-bottom: 24px;
            box-shadow: var(--shadow-lg);
            display: flex;
            align-items: center;
            justify-content: space-between;
        }

        .header h1 {
            color: var(--primary);
            font-size: 2rem;
            font-weight: 700;
        }

        .status-indicator {
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 8px 16px;
            border-radius: 20px;
            font-size: 0.875rem;
            font-weight: 500;
        }

        .status-online { background: var(--success); color: white; }
        .status-offline { background: var(--danger); color: white; }

        .main-grid {
            display: grid;
            grid-template-columns: 300px 1fr;
            gap: 24px;
        }

        .sidebar {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 16px;
            padding: 24px;
            box-shadow: var(--shadow-lg);
            height: fit-content;
        }

        .nav-tabs {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }

        .nav-tab {
            display: flex;
            align-items: center;
            gap: 12px;
            padding: 12px 16px;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.2s;
            color: var(--secondary);
            text-decoration: none;
        }

        .nav-tab:hover, .nav-tab.active {
            background: var(--primary);
            color: white;
        }

        .nav-tab i { font-size: 1.2rem; }

        .content-area {
            display: grid;
            grid-template-rows: auto 1fr;
            gap: 24px;
        }

        .control-panels {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
            gap: 24px;
        }

        .panel {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 16px;
            padding: 24px;
            box-shadow: var(--shadow-lg);
        }

        .panel-header {
            display: flex;
            align-items: center;
            gap: 12px;
            margin-bottom: 20px;
            padding-bottom: 16px;
            border-bottom: 2px solid var(--border);
        }

        .panel-header i {
            font-size: 1.5rem;
            color: var(--primary);
        }

        .panel-header h3 {
            font-size: 1.25rem;
            font-weight: 600;
            color: var(--dark);
        }

        .control-group {
            margin-bottom: 20px;
        }

        .control-group h4 {
            font-size: 1rem;
            font-weight: 500;
            margin-bottom: 12px;
            color: var(--dark);
        }

        .control-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
            gap: 12px;
        }

        .btn {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 8px;
            padding: 16px 12px;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            transition: all 0.2s;
            font-size: 0.875rem;
            font-weight: 500;
            text-align: center;
            min-height: 70px;
            justify-content: center;
        }

        .btn:hover {
            transform: translateY(-2px);
            box-shadow: var(--shadow);
        }

        .btn:active {
            transform: translateY(0);
        }

        .btn-primary { background: var(--primary); color: white; }
        .btn-success { background: var(--success); color: white; }
        .btn-warning { background: var(--warning); color: white; }
        .btn-danger { background: var(--danger); color: white; }
        .btn-secondary { background: var(--secondary); color: white; }

        .slider-group {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }

        .slider {
            -webkit-appearance: none;
            width: 100%;
            height: 8px;
            border-radius: 4px;
            background: var(--border);
            outline: none;
        }

        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: var(--primary);
            cursor: pointer;
        }

        .slider::-moz-range-thumb {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: var(--primary);
            cursor: pointer;
            border: none;
        }

        .value-display {
            font-weight: 600;
            color: var(--primary);
        }

        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 16px;
        }

        .status-card {
            background: var(--light);
            border-radius: 8px;
            padding: 16px;
            border-left: 4px solid var(--primary);
        }

        .status-card h4 {
            font-size: 0.875rem;
            color: var(--secondary);
            margin-bottom: 8px;
        }

        .status-card .value {
            font-size: 1.25rem;
            font-weight: 600;
            color: var(--dark);
        }

        .log-panel {
            background: var(--dark);
            color: #10b981;
            border-radius: 8px;
            padding: 16px;
            font-family: 'JetBrains Mono', monospace;
            font-size: 0.875rem;
            max-height: 300px;
            overflow-y: auto;
            white-space: pre-wrap;
        }

        .log-entry {
            margin-bottom: 4px;
        }

        .log-timestamp {
            color: #64748b;
        }

        .automation-panel {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 16px;
        }

        .automation-card {
            background: var(--light);
            border-radius: 8px;
            padding: 20px;
            text-align: center;
        }

        .automation-card i {
            font-size: 2rem;
            color: var(--primary);
            margin-bottom: 12px;
        }

        .automation-card h4 {
            font-size: 1rem;
            font-weight: 600;
            margin-bottom: 8px;
            color: var(--dark);
        }

        .automation-card p {
            font-size: 0.875rem;
            color: var(--secondary);
            margin-bottom: 16px;
        }

        .form-group {
            margin-bottom: 16px;
        }

        .form-group label {
            display: block;
            font-size: 0.875rem;
            font-weight: 500;
            color: var(--dark);
            margin-bottom: 4px;
        }

        .form-input {
            width: 100%;
            padding: 8px 12px;
            border: 2px solid var(--border);
            border-radius: 6px;
            font-size: 0.875rem;
            transition: border-color 0.2s;
        }

        .form-input:focus {
            outline: none;
            border-color: var(--primary);
        }

        .tab-content {
            display: none;
        }

        .tab-content.active {
            display: block;
        }

        @media (max-width: 1024px) {
            .main-grid {
                grid-template-columns: 1fr;
            }

            .control-panels {
                grid-template-columns: 1fr;
            }
        }

        @media (max-width: 768px) {
            .app-container {
                padding: 16px;
            }

            .header {
                flex-direction: column;
                gap: 16px;
                text-align: center;
            }
        }
    </style>
</head>
<body>
    <div class="app-container">
        <header class="header">
            <h1><i class="fas fa-robot"></i> Autonomous Mobile Manipulator Control Center</h1>
            <div class="status-indicator status-online" id="connection-status">
                <i class="fas fa-circle"></i>
                <span>System Online</span>
            </div>
        </header>

        <div class="main-grid">
            <aside class="sidebar">
                <nav class="nav-tabs">
                    <a href="#movement" class="nav-tab active" onclick="showTab('movement')">
                        <i class="fas fa-arrows-alt"></i>
                        <span>Movement</span>
                    </a>
                    <a href="#manipulation" class="nav-tab" onclick="showTab('manipulation')">
                        <i class="fas fa-hand-paper"></i>
                        <span>Manipulation</span>
                    </a>
                    <a href="#containers" class="nav-tab" onclick="showTab('containers')">
                        <i class="fas fa-boxes"></i>
                        <span>Containers</span>
                    </a>
                    <a href="#automation" class="nav-tab" onclick="showTab('automation')">
                        <i class="fas fa-cogs"></i>
                        <span>Automation</span>
                    </a>
                    <a href="#safety" class="nav-tab" onclick="showTab('safety')">
                        <i class="fas fa-shield-alt"></i>
                        <span>Safety</span>
                    </a>
                    <a href="#status" class="nav-tab" onclick="showTab('status')">
                        <i class="fas fa-chart-line"></i>
                        <span>Status</span>
                    </a>
                    <a href="#hardware" class="nav-tab" onclick="showTab('hardware')">
                        <i class="fas fa-microchip"></i>
                        <span>Hardware</span>
                    </a>
                </nav>
            </aside>

            <main class="content-area">
                <!-- Movement Tab -->
                <div id="movement" class="tab-content active">
                    <div class="control-panels">
                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-gamepad"></i>
                                <h3>Basic Movement Control</h3>
                            </div>

                            <div class="control-group">
                                <h4>Speed Control</h4>
                                <div class="slider-group">
                                    <input type="range" class="slider" id="speed-slider" min="0.1" max="1.0" step="0.1" value="0.5">
                                    <div class="value-display">Speed: <span id="speed-value">0.5</span></div>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Directional Movement</h4>
                                <div class="control-grid">
                                    <button class="btn btn-primary" onclick="moveRobot('forward')">
                                        <i class="fas fa-arrow-up"></i>
                                        <span>Forward</span>
                                    </button>
                                    <button class="btn btn-primary" onclick="moveRobot('backward')">
                                        <i class="fas fa-arrow-down"></i>
                                        <span>Backward</span>
                                    </button>
                                    <button class="btn btn-secondary" onclick="moveRobot('strafe_left')">
                                        <i class="fas fa-arrow-left"></i>
                                        <span>Left</span>
                                    </button>
                                    <button class="btn btn-secondary" onclick="moveRobot('strafe_right')">
                                        <i class="fas fa-arrow-right"></i>
                                        <span>Right</span>
                                    </button>
                                    <button class="btn btn-warning" onclick="stopRobot()">
                                        <i class="fas fa-stop"></i>
                                        <span>Stop</span>
                                    </button>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Rotation</h4>
                                <div class="control-grid">
                                    <button class="btn btn-primary" onclick="turnRobot('left')">
                                        <i class="fas fa-undo"></i>
                                        <span>Turn Left</span>
                                    </button>
                                    <button class="btn btn-primary" onclick="turnRobot('right')">
                                        <i class="fas fa-redo"></i>
                                        <span>Turn Right</span>
                                    </button>
                                </div>
                            </div>
                        </div>

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-cog"></i>
                                <h3>Robot Mode Control</h3>
                            </div>

                            <div class="control-group">
                                <h4>Operating Modes</h4>
                                <div class="control-grid">
                                    <button class="btn btn-success" onclick="setRobotMode('AUTONOMOUS')">
                                        <i class="fas fa-robot"></i>
                                        <span>Autonomous</span>
                                    </button>
                                    <button class="btn btn-primary" onclick="setRobotMode('MANUAL')">
                                        <i class="fas fa-hand-paper"></i>
                                        <span>Manual</span>
                                    </button>
                                    <button class="btn btn-warning" onclick="setRobotMode('MAINTENANCE')">
                                        <i class="fas fa-wrench"></i>
                                        <span>Maintenance</span>
                                    </button>
                                </div>
                            </div>

                            <div class="control-group">
                                <div class="value-display" id="current-mode">Current Mode: <span id="mode-display">UNKNOWN</span></div>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Manipulation Tab -->
                <div id="manipulation" class="tab-content">
                    <div class="control-panels">
                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-hand-paper"></i>
                                <h3>Picker System Control</h3>
                            </div>

                            <div class="control-group">
                                <h4>Gripper Control</h4>
                                <div class="control-grid">
                                    <button class="btn btn-success" onclick="controlGripper('open')">
                                        <i class="fas fa-hand-peace"></i>
                                        <span>Open</span>
                                    </button>
                                    <button class="btn btn-warning" onclick="controlGripper('close')">
                                        <i class="fas fa-hand-rock"></i>
                                        <span>Close</span>
                                    </button>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Gripper Tilt Angle</h4>
                                <div class="slider-group">
                                    <input type="range" class="slider" id="tilt-slider" min="0" max="180" value="90">
                                    <div class="value-display">Angle: <span id="tilt-value">90</span>°</div>
                                    <button class="btn btn-secondary" onclick="setGripperTilt()" style="margin-top: 8px;">
                                        <i class="fas fa-check"></i>
                                        <span>Set Angle</span>
                                    </button>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Gripper Neck Position</h4>
                                <div class="slider-group">
                                    <input type="range" class="slider" id="neck-slider" min="-1" max="1" step="0.1" value="0">
                                    <div class="value-display">Position: <span id="neck-value">0.0</span></div>
                                    <button class="btn btn-secondary" onclick="setGripperNeck()" style="margin-top: 8px;">
                                        <i class="fas fa-check"></i>
                                        <span>Set Position</span>
                                    </button>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Gripper Base Height</h4>
                                <div class="slider-group">
                                    <input type="range" class="slider" id="base-slider" min="0" max="1" step="0.1" value="0.5">
                                    <div class="value-display">Height: <span id="base-value">0.5</span></div>
                                    <button class="btn btn-secondary" onclick="setGripperBase()" style="margin-top: 8px;">
                                        <i class="fas fa-check"></i>
                                        <span>Set Height</span>
                                    </button>
                                </div>
                            </div>
                        </div>

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-home"></i>
                                <h3>Picker System Actions</h3>
                            </div>

                            <div class="control-group">
                                <h4>Quick Actions</h4>
                                <div class="control-grid">
                                    <button class="btn btn-primary" onclick="homePickerSystem()">
                                        <i class="fas fa-home"></i>
                                        <span>Home All</span>
                                    </button>
                                    <button class="btn btn-success" onclick="preparePickup()">
                                        <i class="fas fa-hand-holding"></i>
                                        <span>Pickup Ready</span>
                                    </button>
                                    <button class="btn btn-warning" onclick="preparePlace()">
                                        <i class="fas fa-hand-holding-heart"></i>
                                        <span>Place Ready</span>
                                    </button>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Containers Tab -->
                <div id="containers" class="tab-content">
                    <div class="control-panels">
                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-boxes"></i>
                                <h3>Container Management</h3>
                            </div>

                            <div class="control-group">
                                <h4>Left Side Containers</h4>
                                <div class="control-grid">
                                    <button class="btn btn-primary" onclick="controlContainer('left_front', 'load')">
                                        <i class="fas fa-arrow-up"></i>
                                        <span>Front Load</span>
                                    </button>
                                    <button class="btn btn-secondary" onclick="controlContainer('left_front', 'unload')">
                                        <i class="fas fa-arrow-down"></i>
                                        <span>Front Unload</span>
                                    </button>
                                    <button class="btn btn-primary" onclick="controlContainer('left_back', 'load')">
                                        <i class="fas fa-arrow-up"></i>
                                        <span>Back Load</span>
                                    </button>
                                    <button class="btn btn-secondary" onclick="controlContainer('left_back', 'unload')">
                                        <i class="fas fa-arrow-down"></i>
                                        <span>Back Unload</span>
                                    </button>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Right Side Containers</h4>
                                <div class="control-grid">
                                    <button class="btn btn-success" onclick="controlContainer('right_front', 'load')">
                                        <i class="fas fa-arrow-up"></i>
                                        <span>Front Load</span>
                                    </button>
                                    <button class="btn btn-warning" onclick="controlContainer('right_front', 'unload')">
                                        <i class="fas fa-arrow-down"></i>
                                        <span>Front Unload</span>
                                    </button>
                                    <button class="btn btn-success" onclick="controlContainer('right_back', 'load')">
                                        <i class="fas fa-arrow-up"></i>
                                        <span>Back Load</span>
                                    </button>
                                    <button class="btn btn-warning" onclick="controlContainer('right_back', 'unload')">
                                        <i class="fas fa-arrow-down"></i>
                                        <span>Back Unload</span>
                                    </button>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Automation Tab -->
                <div id="automation" class="tab-content">
                    <div class="automation-panel">
                        <div class="automation-card">
                            <i class="fas fa-route"></i>
                            <h4>Autonomous Patrol</h4>
                            <p>Execute multi-waypoint autonomous navigation</p>
                            <div class="form-group">
                                <label>Waypoints (JSON format):</label>
                                <textarea id="patrol-waypoints" class="form-input" rows="3" placeholder='[{"position": {"x": 0, "y": 0, "z": 0}}, {"position": {"x": 2, "y": 0, "z": 0}}]'></textarea>
                            </div>
                            <div class="form-group">
                                <label>Speed:</label>
                                <input type="number" id="patrol-speed" class="form-input" value="0.5" step="0.1" min="0.1" max="1.0">
                            </div>
                            <button class="btn btn-primary" onclick="executePatrol()">
                                <i class="fas fa-play"></i>
                                <span>Start Patrol</span>
                            </button>
                        </div>

                        <div class="automation-card">
                            <i class="fas fa-eye"></i>
                            <h4>Obstacle Avoidance</h4>
                            <p>Navigate to target while avoiding obstacles</p>
                            <div class="form-group">
                                <label>Target X:</label>
                                <input type="number" id="target-x" class="form-input" value="3.0" step="0.5">
                            </div>
                            <div class="form-group">
                                <label>Target Y:</label>
                                <input type="number" id="target-y" class="form-input" value="1.0" step="0.5">
                            </div>
                            <button class="btn btn-success" onclick="executeObstacleAvoidance()">
                                <i class="fas fa-route"></i>
                                <span>Navigate</span>
                            </button>
                        </div>

                        <div class="automation-card">
                            <i class="fas fa-hand-holding-heart"></i>
                            <h4>Pick & Place</h4>
                            <p>Complete manipulation sequence</p>
                            <div class="form-group">
                                <label>Pickup Location:</label>
                                <input type="text" id="pickup-location" class="form-input" placeholder='{"position": {"x": 1, "y": 0, "z": 0}}'>
                            </div>
                            <div class="form-group">
                                <label>Place Location:</label>
                                <input type="text" id="place-location" class="form-input" placeholder='{"position": {"x": -1, "y": 0, "z": 0}}'>
                            </div>
                            <button class="btn btn-warning" onclick="executePickPlace()">
                                <i class="fas fa-exchange-alt"></i>
                                <span>Execute</span>
                            </button>
                        </div>

                        <div class="automation-card">
                            <i class="fas fa-brain"></i>
                            <h4>n8n Integration</h4>
                            <p>Trigger workflow automation</p>
                            <div class="form-group">
                                <label>Command:</label>
                                <input type="text" id="n8n-command" class="form-input" placeholder="forward">
                            </div>
                            <button class="btn btn-secondary" onclick="triggerN8nWorkflow()">
                                <i class="fas fa-play-circle"></i>
                                <span>Trigger Workflow</span>
                            </button>
                        </div>
                    </div>
                </div>

                <!-- Safety Tab -->
                <div id="safety" class="tab-content">
                    <div class="control-panels">
                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-shield-alt"></i>
                                <h3>Emergency Controls</h3>
                            </div>

                            <div class="control-group">
                                <h4>Emergency Actions</h4>
                                <div class="control-grid">
                                    <button class="btn btn-danger" onclick="emergencyStop()" style="grid-column: span 2;">
                                        <i class="fas fa-exclamation-triangle"></i>
                                        <span>EMERGENCY STOP</span>
                                    </button>
                                    <button class="btn btn-warning" onclick="emergencyStopActivate()">
                                        <i class="fas fa-stop-circle"></i>
                                        <span>Activate Stop</span>
                                    </button>
                                    <button class="btn btn-success" onclick="emergencyStopDeactivate()">
                                        <i class="fas fa-play-circle"></i>
                                        <span>Deactivate Stop</span>
                                    </button>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Status Tab -->
                <div id="status" class="tab-content">
                    <div class="control-panels">
                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-chart-line"></i>
                                <h3>System Status</h3>
                            </div>

                            <div class="status-grid" id="status-grid">
                                <!-- Status will be populated by JavaScript -->
                            </div>
                        </div>

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-terminal"></i>
                                <h3>System Log</h3>
                            </div>

                            <div class="log-panel" id="log-panel"></div>
                        </div>
                    </div>
                </div>

                <!-- Hardware Tab -->
                <div id="hardware" class="tab-content">
                    <div class="control-panels">
                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-microchip"></i>
                                <h3>Hardware Specifications</h3>
                            </div>
                            
                            <div class="control-group">
                                <h4>Robot Configuration</h4>
                                <div class="status-grid">
                                    <div class="status-card">
                                        <h4>Shape</h4>
                                        <div class="value">Hexagonal</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Drive System</h4>
                                        <div class="value">3 Omni Wheels</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Control Platform</h4>
                                        <div class="value">Raspberry Pi 5</div>
                                    </div>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Sensors</h4>
                                <div class="status-grid">
                                    <div class="status-card">
                                        <h4>Laser Distance</h4>
                                        <div class="value">6 units (VL53L0X)</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Ultrasonic</h4>
                                        <div class="value">2x HC-SR04</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Line Sensors</h4>
                                        <div class="value">3 Individual IR</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>TF-Luna LIDAR</h4>
                                        <div class="value">Single-Point</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>IMU</h4>
                                        <div class="value">MPU6050</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Camera</h4>
                                        <div class="value">USB (Gripper)</div>
                                    </div>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Actuators</h4>
                                <div class="status-grid">
                                    <div class="status-card">
                                        <h4>Omni Wheels</h4>
                                        <div class="value">3 Motors</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Gripper System</h4>
                                        <div class="value">4 Components</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Containers</h4>
                                        <div class="value">4 Servos</div>
                                    </div>
                                </div>
                            </div>
                        </div>

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-sitemap"></i>
                                <h3>GPIO Pinout - Raspberry Pi 5</h3>
                            </div>

                            <div class="control-group">
                                <h4>Omni Wheel Motors (3x)</h4>
                                <div class="log-panel" style="max-height: 150px; color: #10b981;">
GPIO17 (11) - Front Left Wheel DIR
GPIO27 (13) - Front Left Wheel PWM
GPIO22 (15) - Front Right Wheel DIR
GPIO23 (16) - Front Right Wheel PWM
GPIO24 (18) - Back Wheel DIR
GPIO25 (22) - Back Wheel PWM
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Gripper System Servos (3x + 1 Motor)</h4>
                                <div class="log-panel" style="max-height: 150px; color: #10b981;">
GPIO18 (12) - Gripper Tilt Servo (PWM)
GPIO19 (35) - Gripper Open/Close Servo (PWM)
GPIO21 (40) - Gripper Extension Servo (360°, PWM)
GPIO12 (32) - Gripper Lifter DIR
GPIO13 (33) - Gripper Lifter PWM
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Laser Distance Sensors (6x VL53L0X)</h4>
                                <div class="log-panel" style="max-height: 150px; color: #10b981;">
GPIO2 (3)  - SDA (I2C Data)
GPIO3 (5)  - SCL (I2C Clock)

TCA9548A I2C Multiplexer Channels:
  CH0: Front Left Laser
  CH1: Front Right Laser
  CH2: Left Front Laser
  CH3: Left Back Laser
  CH4: Right Front Laser
  CH5: Right Back Laser
  CH6: Back Left Laser
  CH7: Back Right Laser
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>HC-SR04 Ultrasonic Sensors (2x)</h4>
                                <div class="log-panel" style="max-height: 100px; color: #10b981;">
GPIO4  (7)  - Front Left TRIG
GPIO14 (8)  - Front Left ECHO (voltage divider)
GPIO15 (10) - Front Right TRIG
GPIO17 (11) - Front Right ECHO (voltage divider)
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Line Sensors (3x IR)</h4>
                                <div class="log-panel" style="max-height: 100px; color: #10b981;">
GPIO5  (29) - Left Line Sensor
GPIO6  (31) - Center Line Sensor
GPIO20 (38) - Right Line Sensor
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Container Load Sensors (4x)</h4>
                                <div class="log-panel" style="max-height: 100px; color: #10b981;">
GPIO7  (26) - Left Front Container
GPIO8  (24) - Left Back Container
GPIO16 (36) - Right Front Container
GPIO26 (37) - Right Back Container
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>IMU & Hardware Controls</h4>
                                <div class="log-panel" style="max-height: 100px; color: #10b981;">
IMU (MPU6050) - I2C Bus 1 (shared, addr 0x68)
  GPIO2 (3)  - SDA
  GPIO3 (5)  - SCL

Hardware Buttons:
  GPIO0 (27) - Emergency Stop
  GPIO1 (28) - Start Button
  GPIO9 (21) - Mode Select
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>USB Interfaces</h4>
                                <div class="log-panel" style="max-height: 80px; color: #10b981;">
USB1 - TF-Luna LIDAR (Serial/USB)
USB2 - USB Camera (Object Recognition)
USB3 - Reserved
                                </div>
                            </div>
                        </div>

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-plug"></i>
                                <h3>Power Distribution</h3>
                            </div>

                            <div class="control-group">
                                <h4>Power Rails</h4>
                                <div class="status-grid">
                                    <div class="status-card">
                                        <h4>Input Voltage</h4>
                                        <div class="value">12V DC</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Current Rating</h4>
                                        <div class="value">5A minimum</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>5V Rail</h4>
                                        <div class="value">RPi + Sensors</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>3.3V Rail</h4>
                                        <div class="value">I2C Devices</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>12V Rail</h4>
                                        <div class="value">Motors</div>
                                    </div>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Safety Notes</h4>
                                <div class="log-panel" style="max-height: 120px; color: #f59e0b;">
⚠️ Use voltage dividers for HC-SR04 ECHO pins (5V → 3.3V)
⚠️ Check all connections before powering on
⚠️ Ensure common ground between all components
⚠️ Use appropriate fuses for motor circuits
⚠️ Emergency stop button must be easily accessible
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </main>
        </div>
    </div>

    <script>
        // Configuration
        const API_BASE = 'http://127.0.0.1:5000';
        let currentSpeed = 0.5;
        let systemStatus = {};

        // Tab switching
        function showTab(tabName) {
            document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));
            document.querySelectorAll('.nav-tab').forEach(tab => tab.classList.remove('active'));

            document.getElementById(tabName).classList.add('active');
            event.target.classList.add('active');
        }

        // API call helper
        async function apiCall(endpoint, method = 'GET', data = null) {
            try {
                const config = {
                    method: method,
                    headers: {
                        'Content-Type': 'application/json'
                    }
                };

                if (data) {
                    config.body = JSON.stringify(data);
                }

                const response = await fetch(`${API_BASE}${endpoint}`, config);
                const result = await response.json();

                addLog(`${method} ${endpoint}: ${result.message || result.success ? 'Success' : 'Failed'}`);
                return result;
            } catch (error) {
                addLog(`ERROR ${endpoint}: ${error.message}`);
                return { success: false, error: error.message };
            }
        }

        // Movement controls
        async function moveRobot(direction) {
            await apiCall('/api/robot/move', 'POST', { direction, speed: currentSpeed });
        }

        async function turnRobot(direction) {
            await apiCall('/api/robot/turn', 'POST', { direction, speed: currentSpeed });
        }

        async function stopRobot() {
            await apiCall('/api/robot/stop', 'POST');
        }

        // Mode control
        async function setRobotMode(mode) {
            const result = await apiCall('/api/robot/mode', 'POST', { mode, reason: 'Web interface request' });
            if (result.success) {
                document.getElementById('mode-display').textContent = mode;
            }
        }

        // Picker system controls
        async function controlGripper(command) {
            await apiCall('/api/robot/picker/gripper', 'POST', { command });
        }

        async function setGripperTilt() {
            const angle = parseInt(document.getElementById('tilt-slider').value);
            await apiCall('/api/robot/picker/gripper_tilt', 'POST', { angle });
        }

        async function setGripperNeck() {
            const position = parseFloat(document.getElementById('neck-slider').value);
            await apiCall('/api/robot/picker/gripper_neck', 'POST', { position });
        }

        async function setGripperBase() {
            const height = parseFloat(document.getElementById('base-slider').value);
            await apiCall('/api/robot/picker/gripper_base', 'POST', { height });
        }

        async function homePickerSystem() {
            await apiCall('/api/robot/servos', 'POST', { action: 'home' });
        }

        async function preparePickup() {
            await controlGripper('open');
            await new Promise(resolve => setTimeout(resolve, 500));
            await setGripperBase();
        }

        async function preparePlace() {
            await controlGripper('close');
            await new Promise(resolve => setTimeout(resolve, 500));
            await setGripperBase();
        }

        // Container controls
        async function controlContainer(containerId, action) {
            await apiCall(`/api/robot/containers/${containerId}`, 'POST', { action });
        }

        // Automation controls
        async function executePatrol() {
            try {
                const waypoints = JSON.parse(document.getElementById('patrol-waypoints').value);
                const speed = parseFloat(document.getElementById('patrol-speed').value);
                await apiCall('/api/robot/patrol', 'POST', { waypoints, patrol_speed: speed });
            } catch (error) {
                addLog('ERROR: Invalid waypoints JSON');
            }
        }

        async function executeObstacleAvoidance() {
            const targetX = parseFloat(document.getElementById('target-x').value);
            const targetY = parseFloat(document.getElementById('target-y').value);
            await apiCall('/api/robot/obstacle-avoidance', 'POST', {
                target_location: { position: { x: targetX, y: targetY, z: 0 } }
            });
        }

        async function executePickPlace() {
            try {
                const pickup = JSON.parse(document.getElementById('pickup-location').value);
                const place = JSON.parse(document.getElementById('place-location').value);
                await apiCall('/api/robot/pick-place', 'POST', { pickup_location: pickup, place_location: place });
            } catch (error) {
                addLog('ERROR: Invalid location JSON');
            }
        }

        async function triggerN8nWorkflow() {
            const command = document.getElementById('n8n-command').value;
            await apiCall('/webhook/robot-control', 'POST', { command });
        }

        // Safety controls
        async function emergencyStop() {
            await apiCall('/api/robot/emergency-stop', 'POST', { activate: true, reason: 'Web interface emergency stop' });
        }

        async function emergencyStopActivate() {
            await apiCall('/api/robot/emergency-stop', 'POST', { activate: true, reason: 'Emergency stop activated' });
        }

        async function emergencyStopDeactivate() {
            await apiCall('/api/robot/emergency-stop', 'POST', { activate: false, reason: 'Emergency stop deactivated' });
        }

        // Status updates
        async function updateStatus() {
            try {
                const response = await fetch(`${API_BASE}/api/robot/status`);
                systemStatus = await response.json();

                if (systemStatus.success) {
                    updateStatusDisplay();
                }
            } catch (error) {
                console.error('Status update failed:', error);
                document.getElementById('connection-status').className = 'status-indicator status-offline';
                document.getElementById('connection-status').innerHTML = '<i class="fas fa-circle"></i><span>Connection Lost</span>';
            }
        }

        function updateStatusDisplay() {
            const statusGrid = document.getElementById('status-grid');
            const data = systemStatus.data || {};

            statusGrid.innerHTML = `
                <div class="status-card">
                    <h4>Robot Mode</h4>
                    <div class="value">${data.mode || 'UNKNOWN'}</div>
                </div>
                <div class="status-card">
                    <h4>Position</h4>
                    <div class="value">X: ${(data.current_pose?.position?.x || 0).toFixed(2)}, Y: ${(data.current_pose?.position?.y || 0).toFixed(2)}</div>
                </div>
                <div class="status-card">
                    <h4>Velocity</h4>
                    <div class="value">${(data.current_velocity?.linear?.x || 0).toFixed(2)} m/s</div>
                </div>
                <div class="status-card">
                    <h4>Safety Status</h4>
                    <div class="value">${data.emergency_stop_active ? 'EMERGENCY' : 'NORMAL'}</div>
                </div>
                <div class="status-card">
                    <h4>System Health</h4>
                    <div class="value">${data.safety_systems_ok ? 'OK' : 'ERROR'}</div>
                </div>
                <div class="status-card">
                    <h4>Battery</h4>
                    <div class="value">${data.battery_level || 0}%</div>
                </div>
            `;

            // Update connection status
            document.getElementById('connection-status').className = 'status-indicator status-online';
            document.getElementById('connection-status').innerHTML = '<i class="fas fa-circle"></i><span>System Online</span>';

            // Update current mode display
            document.getElementById('mode-display').textContent = data.mode || 'UNKNOWN';
        }

        // Slider value updates
        document.getElementById('speed-slider').addEventListener('input', function() {
            currentSpeed = parseFloat(this.value);
            document.getElementById('speed-value').textContent = currentSpeed.toFixed(1);
        });

        document.getElementById('tilt-slider').addEventListener('input', function() {
            document.getElementById('tilt-value').textContent = this.value;
        });

        document.getElementById('neck-slider').addEventListener('input', function() {
            document.getElementById('neck-value').textContent = parseFloat(this.value).toFixed(1);
        });

        document.getElementById('base-slider').addEventListener('input', function() {
            document.getElementById('base-value').textContent = parseFloat(this.value).toFixed(1);
        });

        // Logging
        function addLog(message) {
            const logPanel = document.getElementById('log-panel');
            const timestamp = new Date().toLocaleTimeString();
            const logEntry = document.createElement('div');
            logEntry.className = 'log-entry';
            logEntry.innerHTML = `<span class="log-timestamp">[${timestamp}]</span> ${message}`;
            logPanel.appendChild(logEntry);
            logPanel.scrollTop = logPanel.scrollHeight;
        }

        // Initialize
        addLog('Autonomous Mobile Manipulator Control Center initialized');
        updateStatus();

        // Update status every 2 seconds
        setInterval(updateStatus, 2000);
    </script>
</body>
</html>
"""

class WebRobotInterface(Node):
    def __init__(self):
        super().__init__('web_robot_interface')

        # Flask app for the professional web interface
        self.app = Flask(__name__)

        # Setup routes
        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)

        @self.app.route('/health')
        def health():
            return jsonify({
                'status': 'healthy',
                'service': 'robot_web_interface',
                'timestamp': time.time()
            })

        self.get_logger().info('Professional Robot Web Interface initialized')
        self.get_logger().info('Access the interface at: http://localhost:8000')

    def run_web_interface(self):
        """Run the web interface in a separate thread"""
        def run_server():
            self.app.run(host='0.0.0.0', port=8000, debug=False, threaded=True)

        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

def main(args=None):
    rclpy.init(args=args)

    try:
        web_interface = WebRobotInterface()
        web_interface.run_web_interface()

        # Keep the node alive
        rclpy.spin(web_interface)

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
