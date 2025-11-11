#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, render_template_string, jsonify, request
import threading
import time
import spidev
import math

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

        .activity-log-footer {
            margin-top: 20px;
            background: var(--dark);
            border-radius: 8px;
            padding: 12px;
            border: 1px solid #374151;
        }

        .activity-log-header {
            display: flex;
            align-items: center;
            gap: 8px;
            margin-bottom: 8px;
            color: #10b981;
            font-weight: 600;
            font-size: 0.875rem;
        }

        .activity-log-content {
            background: #0f172a;
            border-radius: 4px;
            padding: 8px;
            font-family: 'JetBrains Mono', monospace;
            font-size: 0.75rem;
            max-height: 150px;
            overflow-y: auto;
            color: #10b981;
        }

        .activity-log-content .log-entry {
            margin-bottom: 2px;
            line-height: 1.4;
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

        .waypoint-container {
            background: var(--dark);
            border-radius: 8px;
            padding: 8px;
            max-height: 300px;
            overflow-y: auto;
        }

        .waypoint-item {
            background: var(--light);
            border-radius: 6px;
            padding: 12px;
            margin-bottom: 8px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-left: 3px solid var(--primary);
        }

        .waypoint-info {
            flex: 1;
        }

        .waypoint-coords {
            font-size: 0.875rem;
            color: var(--secondary);
            font-family: 'JetBrains Mono', monospace;
        }

        .waypoint-actions button {
            background: none;
            border: none;
            color: var(--danger);
            cursor: pointer;
            padding: 4px 8px;
            border-radius: 4px;
            transition: background 0.2s;
        }

        .waypoint-actions button:hover {
            background: rgba(239, 68, 68, 0.1);
        }

        .input-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 12px;
            margin-bottom: 12px;
        }

        .input-group {
            display: flex;
            flex-direction: column;
        }

        .input-group label {
            font-size: 0.875rem;
            color: var(--secondary);
            margin-bottom: 4px;
        }

        .input-field {
            background: var(--light);
            border: 1px solid #374151;
            border-radius: 6px;
            padding: 8px 12px;
            color: var(--dark);
            font-size: 0.875rem;
        }

        .input-field:focus {
            outline: none;
            border-color: var(--primary);
        }

        .checkbox-group {
            margin: 12px 0;
        }

        .checkbox-group label {
            display: flex;
            align-items: center;
            color: var(--secondary);
            cursor: pointer;
        }

        .checkbox-group input[type="checkbox"] {
            margin-right: 8px;
            width: 18px;
            height: 18px;
            cursor: pointer;
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
                    <a href="#path-planning" class="nav-tab" onclick="showTab('path-planning')">
                        <i class="fas fa-route"></i>
                        <span>Path Planning</span>
                    </a>
                    <a href="#sensors" class="nav-tab" onclick="showTab('sensors')">
                        <i class="fas fa-radar"></i>
                        <span>Sensors</span>
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

                        <!-- Activity Log -->
                        <div class="activity-log-footer">
                            <div class="activity-log-header">
                                <i class="fas fa-stream"></i>
                                <span>Activity Stream</span>
                            </div>
                            <div class="activity-log-content" id="log-panel"></div>
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

                        <!-- Activity Log -->
                        <div class="activity-log-footer">
                            <div class="activity-log-header">
                                <i class="fas fa-stream"></i>
                                <span>Activity Stream</span>
                            </div>
                            <div class="activity-log-content" id="log-panel-gripper"></div>
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

                        <!-- Activity Log -->
                        <div class="activity-log-footer">
                            <div class="activity-log-header">
                                <i class="fas fa-stream"></i>
                                <span>Activity Stream</span>
                            </div>
                            <div class="activity-log-content" id="log-panel-containers"></div>
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

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-history"></i>
                                <h3>Last 3 Commands</h3>
                            </div>

                            <div class="log-panel" id="commands-panel" style="max-height: 200px;">Loading...</div>
                        </div>

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-clipboard-list"></i>
                                <h3>Robot System Log</h3>
                            </div>

                            <div class="log-panel" id="robot-log-panel" style="max-height: 200px;">Loading...</div>
                        </div>

                        <!-- Activity Log -->
                        <div class="activity-log-footer">
                            <div class="activity-log-header">
                                <i class="fas fa-stream"></i>
                                <span>Activity Stream</span>
                            </div>
                            <div class="activity-log-content" id="log-panel-status"></div>
                        </div>
                    </div>
                </div>

                <!-- Sensors Tab -->
                <div id="sensors" class="tab-content">
                    <div class="control-panels">
                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-ruler-combined"></i>
                                <h3>IR Distance Sensors (6x Sharp GP2Y0A02YK0F)</h3>
                            </div>

                            <div class="status-grid" id="laser-sensors">
                                <div class="status-card">
                                    <h4>Left Front</h4>
                                    <div class="value" id="laser-left-front">-- mm</div>
                                </div>
                                <div class="status-card">
                                    <h4>Left Back</h4>
                                    <div class="value" id="laser-left-back">-- mm</div>
                                </div>
                                <div class="status-card">
                                    <h4>Right Front</h4>
                                    <div class="value" id="laser-right-front">-- mm</div>
                                </div>
                                <div class="status-card">
                                    <h4>Right Back</h4>
                                    <div class="value" id="laser-right-back">-- mm</div>
                                </div>
                                <div class="status-card">
                                    <h4>Back Left</h4>
                                    <div class="value" id="laser-back-left">-- mm</div>
                                </div>
                                <div class="status-card">
                                    <h4>Back Right</h4>
                                    <div class="value" id="laser-back-right">-- mm</div>
                                </div>
                            </div>
                        </div>

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-broadcast-tower"></i>
                                <h3>Ultrasonic & LIDAR Sensors</h3>
                            </div>

                            <div class="control-group">
                                <h4>Ultrasonic Sensors (HC-SR04)</h4>
                                <div class="status-grid">
                                    <div class="status-card">
                                        <h4>Front Left</h4>
                                        <div class="value" id="ultrasonic-front-left">-- cm</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Front Right</h4>
                                        <div class="value" id="ultrasonic-front-right">-- cm</div>
                                    </div>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>TF-Luna LIDAR</h4>
                                <div class="status-grid">
                                    <div class="status-card">
                                        <h4>Distance</h4>
                                        <div class="value" id="tf-luna-distance">-- cm</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Signal Strength</h4>
                                        <div class="value" id="tf-luna-strength">--</div>
                                    </div>
                                </div>
                            </div>
                        </div>

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-road"></i>
                                <h3>Line Sensors & IMU</h3>
                            </div>

                            <div class="control-group">
                                <h4>Line Sensors (3x IR)</h4>
                                <div class="status-grid">
                                    <div class="status-card">
                                        <h4>Left</h4>
                                        <div class="value" id="line-left">--</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Center</h4>
                                        <div class="value" id="line-center">--</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Right</h4>
                                        <div class="value" id="line-right">--</div>
                                    </div>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>IMU (MPU6050)</h4>
                                <div class="status-grid">
                                    <div class="status-card">
                                        <h4>Orientation X</h4>
                                        <div class="value" id="imu-orient-x">--°</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Orientation Y</h4>
                                        <div class="value" id="imu-orient-y">--°</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Orientation Z</h4>
                                        <div class="value" id="imu-orient-z">--°</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Angular Velocity</h4>
                                        <div class="value" id="imu-angular">-- rad/s</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Linear Accel X</h4>
                                        <div class="value" id="imu-accel-x">-- m/s²</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Linear Accel Y</h4>
                                        <div class="value" id="imu-accel-y">-- m/s²</div>
                                    </div>
                                </div>
                                <div style="margin-top: 16px;">
                                    <button class="control-btn warning" onclick="calibrateIMU()" style="width: 100%;">
                                        <i class="fas fa-crosshairs"></i> Calibrate IMU (Set Zero)
                                    </button>
                                    <div id="imu-calibration-status" style="margin-top: 8px; text-align: center; color: #10b981; font-size: 0.875rem;"></div>
                                </div>
                            </div>
                        </div>

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-boxes"></i>
                                <h3>Container Load Sensors</h3>
                            </div>

                            <div class="status-grid">
                                <div class="status-card">
                                    <h4>Left Front</h4>
                                    <div class="value" id="container-left-front">Empty</div>
                                </div>
                                <div class="status-card">
                                    <h4>Left Back</h4>
                                    <div class="value" id="container-left-back">Empty</div>
                                </div>
                                <div class="status-card">
                                    <h4>Right Front</h4>
                                    <div class="value" id="container-right-front">Empty</div>
                                </div>
                                <div class="status-card">
                                    <h4>Right Back</h4>
                                    <div class="value" id="container-right-back">Empty</div>
                                </div>
                            </div>
                        </div>

                        <!-- Activity Log -->
                        <div class="activity-log-footer">
                            <div class="activity-log-header">
                                <i class="fas fa-stream"></i>
                                <span>Activity Stream</span>
                            </div>
                            <div class="activity-log-content" id="log-panel-sensors"></div>
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
                                        <h4>IR Distance Sensors</h4>
                                        <div class="value">6 units (Sharp GP2Y0A02YK0F)</div>
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
                                <h4>IR Distance Sensors (6x Sharp GP2Y0A02YK0F)</h4>
                                <div class="log-panel" style="max-height: 150px; color: #10b981;">
Analog sensors connected via MCP3008 ADC (SPI interface)

SPI0 Pinout:
GPIO9  (21) - MISO (SPI0_MISO)
GPIO10 (19) - MOSI (SPI0_MOSI)
GPIO11 (23) - SCLK (SPI0_SCLK)
GPIO8  (24) - CE0  (SPI0_CE0)

MCP3008 ADC Channel Mapping:
  CH0: Left Front Sensor
  CH1: Left Back Sensor
  CH2: Right Front Sensor
  CH3: Right Back Sensor
  CH4: Back Left Sensor
  CH5: Back Right Sensor
  
Sensor Specifications:
- Range: 20-150cm (200-1500mm)
- Output: Analog voltage 0.4V-2.7V
- Power: 5V supply required
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

                        <!-- Activity Log -->
                        <div class="activity-log-footer">
                            <div class="activity-log-header">
                                <i class="fas fa-stream"></i>
                                <span>Activity Stream</span>
                            </div>
                            <div class="activity-log-content" id="log-panel-hardware"></div>
                        </div>
                    </div>
                </div>

                <!-- Path Planning Tab -->
                <div id="path-planning" class="tab-content">
                    <div class="control-panels">
                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-map-marked-alt"></i>
                                <span>Waypoint Manager</span>
                            </div>
                            
                            <div class="control-group">
                                <h4>Add Waypoint</h4>
                                <div class="input-grid">
                                    <div class="input-group">
                                        <label>X Position (m)</label>
                                        <input type="number" id="waypoint-x" step="0.1" value="0.0" class="input-field">
                                    </div>
                                    <div class="input-group">
                                        <label>Y Position (m)</label>
                                        <input type="number" id="waypoint-y" step="0.1" value="0.0" class="input-field">
                                    </div>
                                    <div class="input-group">
                                        <label>Z Position (m)</label>
                                        <input type="number" id="waypoint-z" step="0.1" value="0.0" class="input-field">
                                    </div>
                                </div>
                                <button class="control-btn" onclick="addWaypoint()">
                                    <i class="fas fa-plus"></i> Add Waypoint
                                </button>
                            </div>

                            <div class="control-group">
                                <h4>Quick Patterns</h4>
                                <div class="button-grid" style="grid-template-columns: repeat(2, 1fr);">
                                    <button class="control-btn" onclick="loadSquarePattern()">
                                        <i class="fas fa-square"></i> Square (2x2m)
                                    </button>
                                    <button class="control-btn" onclick="loadTrianglePattern()">
                                        <i class="fas fa-play"></i> Triangle
                                    </button>
                                    <button class="control-btn" onclick="loadHexagonPattern()">
                                        <i class="fas fa-hexagon"></i> Hexagon
                                    </button>
                                    <button class="control-btn" onclick="loadLinePattern()">
                                        <i class="fas fa-minus"></i> Straight Line
                                    </button>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Waypoint List</h4>
                                <div id="waypoint-list" class="waypoint-container">
                                    <!-- Waypoints will be added here -->
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Path Execution Settings</h4>
                                <div class="input-grid">
                                    <div class="input-group">
                                        <label>Speed (m/s)</label>
                                        <input type="number" id="patrol-speed" step="0.1" value="0.5" min="0.1" max="2.0" class="input-field">
                                    </div>
                                    <div class="input-group">
                                        <label>Cycles</label>
                                        <input type="number" id="patrol-cycles" step="1" value="1" min="1" max="10" class="input-field">
                                    </div>
                                </div>
                                <div class="checkbox-group">
                                    <label>
                                        <input type="checkbox" id="return-to-start" checked>
                                        Return to Start Position
                                    </label>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Execute Path</h4>
                                <div class="button-grid" style="grid-template-columns: 1fr 1fr;">
                                    <button class="control-btn success" onclick="executePatrol()">
                                        <i class="fas fa-play"></i> Start Patrol
                                    </button>
                                    <button class="control-btn danger" onclick="clearWaypoints()">
                                        <i class="fas fa-trash"></i> Clear All
                                    </button>
                                </div>
                            </div>
                        </div>

                        <div class="panel">
                            <div class="panel-header">
                                <i class="fas fa-compass"></i>
                                <span>Path Status & Visualization</span>
                            </div>

                            <div class="control-group">
                                <h4>Current Path Execution</h4>
                                <div class="status-grid">
                                    <div class="status-card">
                                        <h4>Status</h4>
                                        <div class="value" id="path-status">Idle</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Waypoints</h4>
                                        <div class="value" id="path-waypoint-count">0</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Progress</h4>
                                        <div class="value" id="path-progress">0%</div>
                                    </div>
                                    <div class="status-card">
                                        <h4>Distance</h4>
                                        <div class="value" id="path-distance">0.0m</div>
                                    </div>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Path Visualization</h4>
                                <canvas id="path-canvas" width="600" height="600" style="border: 2px solid #374151; border-radius: 8px; background: #1f2937; width: 100%; max-width: 600px;"></canvas>
                            </div>

                            <div class="control-group">
                                <h4>Navigation Controls</h4>
                                <div class="button-grid" style="grid-template-columns: repeat(2, 1fr);">
                                    <button class="control-btn" onclick="goToWaypoint()">
                                        <i class="fas fa-map-pin"></i> Go to Point
                                    </button>
                                    <button class="control-btn" onclick="returnHome()">
                                        <i class="fas fa-home"></i> Return Home
                                    </button>
                                    <button class="control-btn warning" onclick="pausePatrol()">
                                        <i class="fas fa-pause"></i> Pause
                                    </button>
                                    <button class="control-btn danger" onclick="stopPatrol()">
                                        <i class="fas fa-stop"></i> Stop
                                    </button>
                                </div>
                            </div>

                            <div class="control-group">
                                <h4>Saved Paths</h4>
                                <div class="input-grid">
                                    <div class="input-group">
                                        <label>Path Name</label>
                                        <input type="text" id="path-name" placeholder="Enter path name" class="input-field">
                                    </div>
                                </div>
                                <div class="button-grid" style="grid-template-columns: 1fr 1fr;">
                                    <button class="control-btn" onclick="savePath()">
                                        <i class="fas fa-save"></i> Save Path
                                    </button>
                                    <button class="control-btn" onclick="loadPath()">
                                        <i class="fas fa-folder-open"></i> Load Path
                                    </button>
                                </div>
                                <div id="saved-paths-list" class="waypoint-container" style="margin-top: 10px;">
                                    <!-- Saved paths will appear here -->
                                </div>
                            </div>
                        </div>

                        <!-- Activity Log -->
                        <div class="activity-log-footer">
                            <div class="activity-log-header">
                                <i class="fas fa-stream"></i>
                                <span>Activity Stream</span>
                            </div>
                            <div class="activity-log-content" id="log-panel-path"></div>
                        </div>
                    </div>
                </div>
            </main>
        </div>
    </div>

    <script>
        // Configuration - Use current hostname for API calls
        // This allows access from any device on the network
        const API_BASE = `http://${window.location.hostname}:5000`;
        let currentSpeed = 0.5;
        let systemStatus = {};
        
        // Log the API endpoint being used
        console.log(`API Base URL: ${API_BASE}`);

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

        // Sensor data updates
        async function updateSensorData() {
            try {
                const response = await fetch(`${API_BASE}/api/robot/sensors`);
                const sensorData = await response.json();

                if (sensorData.success && sensorData.data) {
                    const data = sensorData.data;

                    // Laser distance sensors
                    if (data.laser_sensors) {
                        document.getElementById('laser-left-front').textContent = (data.laser_sensors.left_front || '--') + ' mm';
                        document.getElementById('laser-left-back').textContent = (data.laser_sensors.left_back || '--') + ' mm';
                        document.getElementById('laser-right-front').textContent = (data.laser_sensors.right_front || '--') + ' mm';
                        document.getElementById('laser-right-back').textContent = (data.laser_sensors.right_back || '--') + ' mm';
                        document.getElementById('laser-back-left').textContent = (data.laser_sensors.back_left || '--') + ' mm';
                        document.getElementById('laser-back-right').textContent = (data.laser_sensors.back_right || '--') + ' mm';
                    }

                    // Ultrasonic sensors
                    if (data.ultrasonic_sensors) {
                        document.getElementById('ultrasonic-front-left').textContent = (data.ultrasonic_sensors.front_left || '--') + ' cm';
                        document.getElementById('ultrasonic-front-right').textContent = (data.ultrasonic_sensors.front_right || '--') + ' cm';
                    }

                    // TF-Luna LIDAR
                    if (data.tf_luna) {
                        document.getElementById('tf-luna-distance').textContent = (data.tf_luna.distance || '--') + ' cm';
                        document.getElementById('tf-luna-strength').textContent = data.tf_luna.strength || '--';
                    }

                    // Line sensors
                    if (data.line_sensors) {
                        document.getElementById('line-left').textContent = data.line_sensors.left ? 'DETECTED' : 'No Line';
                        document.getElementById('line-center').textContent = data.line_sensors.center ? 'DETECTED' : 'No Line';
                        document.getElementById('line-right').textContent = data.line_sensors.right ? 'DETECTED' : 'No Line';
                    }

                    // Container load sensors
                    if (data.container_sensors) {
                        document.getElementById('container-left-front').textContent = data.container_sensors.left_front ? 'Loaded' : 'Empty';
                        document.getElementById('container-left-back').textContent = data.container_sensors.left_back ? 'Loaded' : 'Empty';
                        document.getElementById('container-right-front').textContent = data.container_sensors.right_front ? 'Loaded' : 'Empty';
                        document.getElementById('container-right-back').textContent = data.container_sensors.right_back ? 'Loaded' : 'Empty';
                    }
                }
            } catch (error) {
                console.error('Sensor data update failed:', error);
            }
        }

        // IMU data updates
        async function updateIMUData() {
            try {
                const response = await fetch(`${API_BASE}/api/robot/imu/position`);
                const imuData = await response.json();

                if (imuData.success && imuData.data) {
                    const data = imuData.data;

                    // Orientation (convert quaternion to euler angles if needed)
                    if (data.orientation) {
                        document.getElementById('imu-orient-x').textContent = (data.orientation.x || 0).toFixed(2) + '°';
                        document.getElementById('imu-orient-y').textContent = (data.orientation.y || 0).toFixed(2) + '°';
                        document.getElementById('imu-orient-z').textContent = (data.orientation.z || 0).toFixed(2) + '°';
                    }

                    // Angular velocity
                    if (data.angular_velocity) {
                        const angVel = Math.sqrt(
                            Math.pow(data.angular_velocity.x || 0, 2) +
                            Math.pow(data.angular_velocity.y || 0, 2) +
                            Math.pow(data.angular_velocity.z || 0, 2)
                        );
                        document.getElementById('imu-angular').textContent = angVel.toFixed(3) + ' rad/s';
                    }

                    // Linear acceleration
                    if (data.linear_acceleration) {
                        document.getElementById('imu-accel-x').textContent = (data.linear_acceleration.x || 0).toFixed(2) + ' m/s²';
                        document.getElementById('imu-accel-y').textContent = (data.linear_acceleration.y || 0).toFixed(2) + ' m/s²';
                    }
                }
            } catch (error) {
                console.error('IMU data update failed:', error);
            }
        }

        // Last commands update
        async function updateLastCommands() {
            try {
                const response = await fetch(`${API_BASE}/api/robot/commands/last`);
                const commandData = await response.json();

                if (commandData.success && commandData.data && commandData.data.commands) {
                    const panel = document.getElementById('commands-panel');
                    panel.innerHTML = '';
                    
                    commandData.data.commands.forEach((cmd, index) => {
                        const cmdEntry = document.createElement('div');
                        cmdEntry.className = 'log-entry';
                        cmdEntry.innerHTML = `<span class="log-timestamp">[${cmd.timestamp || 'N/A'}]</span> ${cmd.command || 'Unknown'} - ${cmd.status || 'N/A'}`;
                        panel.appendChild(cmdEntry);
                    });

                    if (commandData.data.commands.length === 0) {
                        panel.textContent = 'No commands executed yet';
                    }
                }
            } catch (error) {
                console.error('Commands update failed:', error);
                document.getElementById('commands-panel').textContent = 'Failed to load commands';
            }
        }

        // Robot log update
        async function updateRobotLog() {
            try {
                const response = await fetch(`${API_BASE}/api/robot/log`);
                const logData = await response.json();

                if (logData.success && logData.data && logData.data.logs) {
                    const panel = document.getElementById('robot-log-panel');
                    panel.innerHTML = '';
                    
                    logData.data.logs.slice(0, 10).forEach((log, index) => {
                        const logEntry = document.createElement('div');
                        logEntry.className = 'log-entry';
                        logEntry.innerHTML = `<span class="log-timestamp">[${log.timestamp || 'N/A'}]</span> ${log.message || log}`;
                        panel.appendChild(logEntry);
                    });

                    if (logData.data.logs.length === 0) {
                        panel.textContent = 'No logs available';
                    }
                }
            } catch (error) {
                console.error('Robot log update failed:', error);
                document.getElementById('robot-log-panel').textContent = 'Failed to load logs';
            }
        }

        // IMU Calibration
        async function calibrateIMU() {
            try {
                const statusDiv = document.getElementById('imu-calibration-status');
                statusDiv.textContent = 'Calibrating IMU...';
                statusDiv.style.color = '#f59e0b';

                const response = await fetch(`${API_BASE}/api/robot/imu/calibrate`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' }
                });

                const result = await response.json();
                
                if (result.success) {
                    statusDiv.textContent = '✓ IMU Calibrated - Zero reference set';
                    statusDiv.style.color = '#10b981';
                    addLog('IMU calibration completed successfully');
                    
                    // Clear message after 3 seconds
                    setTimeout(() => {
                        statusDiv.textContent = '';
                    }, 3000);
                } else {
                    statusDiv.textContent = '✗ Calibration failed: ' + (result.error || 'Unknown error');
                    statusDiv.style.color = '#ef4444';
                    addLog('IMU calibration failed: ' + result.error);
                }
            } catch (error) {
                const statusDiv = document.getElementById('imu-calibration-status');
                statusDiv.textContent = '✗ Calibration error: ' + error.message;
                statusDiv.style.color = '#ef4444';
                addLog('IMU calibration error: ' + error.message);
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
            const timestamp = new Date().toLocaleTimeString();
            const logHTML = `<span class="log-timestamp">[${timestamp}]</span> ${message}`;
            
            // Add to all activity log panels across all tabs
            const logPanels = [
                'log-panel',           // Movement tab
                'log-panel-gripper',   // Gripper tab
                'log-panel-containers',// Containers tab
                'log-panel-status',    // Status tab
                'log-panel-path',      // Path Planning tab
                'log-panel-sensors',   // Sensors tab
                'log-panel-hardware'   // Hardware tab
            ];
            
            logPanels.forEach(panelId => {
                const panel = document.getElementById(panelId);
                if (panel) {
                    const logEntry = document.createElement('div');
                    logEntry.className = 'log-entry';
                    logEntry.innerHTML = logHTML;
                    panel.appendChild(logEntry);
                    
                    // Keep only last 50 entries per panel
                    while (panel.children.length > 50) {
                        panel.removeChild(panel.firstChild);
                    }
                    
                    panel.scrollTop = panel.scrollHeight;
                }
            });
        }

        // Initialize
        addLog('Autonomous Mobile Manipulator Control Center initialized');
        updateStatus();
        updateSensorData();
        updateIMUData();
        updateLastCommands();
        updateRobotLog();

        // Update status every 2 seconds
        setInterval(updateStatus, 2000);
        
        // Update sensor data every 1 second
        setInterval(updateSensorData, 1000);
        
        // Update IMU data every 500ms for smooth updates
        setInterval(updateIMUData, 500);
        
        // Update commands and logs every 3 seconds
        setInterval(updateLastCommands, 3000);
        setInterval(updateRobotLog, 3000);

        // Path Planning Functions
        let waypoints = [];
        let savedPaths = JSON.parse(localStorage.getItem('robotPaths') || '{}');
        let pathCanvas = null;
        let pathContext = null;

        // Initialize canvas when path-planning tab is shown
        function initializeCanvas() {
            if (!pathCanvas) {
                pathCanvas = document.getElementById('path-canvas');
                if (pathCanvas) {
                    pathContext = pathCanvas.getContext('2d');
                    updatePathVisualization();
                }
            }
        }

        // Add waypoint
        function addWaypoint() {
            const x = parseFloat(document.getElementById('waypoint-x').value);
            const y = parseFloat(document.getElementById('waypoint-y').value);
            const z = parseFloat(document.getElementById('waypoint-z').value);

            waypoints.push({ position: { x, y, z } });
            updateWaypointList();
            updatePathVisualization();
            addLog(`Waypoint added: (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)})`);
        }

        // Remove waypoint
        function removeWaypoint(index) {
            waypoints.splice(index, 1);
            updateWaypointList();
            updatePathVisualization();
            addLog(`Waypoint ${index + 1} removed`);
        }

        // Update waypoint list display
        function updateWaypointList() {
            const listContainer = document.getElementById('waypoint-list');
            listContainer.innerHTML = '';

            if (waypoints.length === 0) {
                listContainer.innerHTML = '<div style="color: #64748b; text-align: center; padding: 20px;">No waypoints added yet</div>';
                document.getElementById('path-waypoint-count').textContent = '0';
                return;
            }

            waypoints.forEach((wp, index) => {
                const item = document.createElement('div');
                item.className = 'waypoint-item';
                item.innerHTML = `
                    <div class="waypoint-info">
                        <div style="font-weight: 600; margin-bottom: 4px;">Waypoint ${index + 1}</div>
                        <div class="waypoint-coords">x: ${wp.position.x.toFixed(2)}, y: ${wp.position.y.toFixed(2)}, z: ${wp.position.z.toFixed(2)}</div>
                    </div>
                    <div class="waypoint-actions">
                        <button onclick="removeWaypoint(${index})"><i class="fas fa-times"></i></button>
                    </div>
                `;
                listContainer.appendChild(item);
            });

            document.getElementById('path-waypoint-count').textContent = waypoints.length;
        }

        // Clear all waypoints
        function clearWaypoints() {
            waypoints = [];
            updateWaypointList();
            updatePathVisualization();
            addLog('All waypoints cleared');
        }

        // Load pattern functions
        function loadSquarePattern() {
            waypoints = [
                { position: { x: 2.0, y: 0.0, z: 0.0 } },
                { position: { x: 2.0, y: 2.0, z: 0.0 } },
                { position: { x: 0.0, y: 2.0, z: 0.0 } },
                { position: { x: 0.0, y: 0.0, z: 0.0 } }
            ];
            updateWaypointList();
            updatePathVisualization();
            addLog('Loaded square pattern (2x2m)');
        }

        function loadTrianglePattern() {
            waypoints = [
                { position: { x: 2.0, y: 0.0, z: 0.0 } },
                { position: { x: 1.0, y: 2.0, z: 0.0 } },
                { position: { x: 0.0, y: 0.0, z: 0.0 } }
            ];
            updateWaypointList();
            updatePathVisualization();
            addLog('Loaded triangle pattern');
        }

        function loadHexagonPattern() {
            const radius = 1.5;
            waypoints = [];
            for (let i = 0; i < 6; i++) {
                const angle = (Math.PI / 3) * i;
                waypoints.push({
                    position: {
                        x: radius * Math.cos(angle),
                        y: radius * Math.sin(angle),
                        z: 0.0
                    }
                });
            }
            updateWaypointList();
            updatePathVisualization();
            addLog('Loaded hexagon pattern');
        }

        function loadLinePattern() {
            waypoints = [
                { position: { x: 0.0, y: 0.0, z: 0.0 } },
                { position: { x: 3.0, y: 0.0, z: 0.0 } }
            ];
            updateWaypointList();
            updatePathVisualization();
            addLog('Loaded straight line pattern (3m)');
        }

        // Visualize path on canvas
        function updatePathVisualization() {
            initializeCanvas();
            if (!pathContext) return;

            const canvas = pathCanvas;
            const ctx = pathContext;

            // Clear canvas
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // Draw grid
            ctx.strokeStyle = '#374151';
            ctx.lineWidth = 1;
            const gridSize = 50;
            for (let i = 0; i < canvas.width; i += gridSize) {
                ctx.beginPath();
                ctx.moveTo(i, 0);
                ctx.lineTo(i, canvas.height);
                ctx.stroke();
            }
            for (let i = 0; i < canvas.height; i += gridSize) {
                ctx.beginPath();
                ctx.moveTo(0, i);
                ctx.lineTo(canvas.width, i);
                ctx.stroke();
            }

            // Draw origin
            const centerX = canvas.width / 2;
            const centerY = canvas.height / 2;
            const scale = 50; // 50 pixels = 1 meter

            ctx.strokeStyle = '#10b981';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(centerX - 20, centerY);
            ctx.lineTo(centerX + 20, centerY);
            ctx.moveTo(centerX, centerY - 20);
            ctx.lineTo(centerX, centerY + 20);
            ctx.stroke();

            if (waypoints.length === 0) return;

            // Draw path
            ctx.strokeStyle = '#3b82f6';
            ctx.lineWidth = 3;
            ctx.beginPath();
            waypoints.forEach((wp, index) => {
                const x = centerX + (wp.position.x * scale);
                const y = centerY - (wp.position.y * scale);
                if (index === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            });
            ctx.stroke();

            // Draw waypoints
            waypoints.forEach((wp, index) => {
                const x = centerX + (wp.position.x * scale);
                const y = centerY - (wp.position.y * scale);

                // Waypoint circle
                ctx.fillStyle = '#3b82f6';
                ctx.beginPath();
                ctx.arc(x, y, 8, 0, 2 * Math.PI);
                ctx.fill();

                // Waypoint number
                ctx.fillStyle = '#ffffff';
                ctx.font = 'bold 12px sans-serif';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText((index + 1).toString(), x, y);
            });

            // Calculate total distance
            let totalDistance = 0;
            for (let i = 1; i < waypoints.length; i++) {
                const dx = waypoints[i].position.x - waypoints[i-1].position.x;
                const dy = waypoints[i].position.y - waypoints[i-1].position.y;
                totalDistance += Math.sqrt(dx * dx + dy * dy);
            }
            document.getElementById('path-distance').textContent = totalDistance.toFixed(2) + 'm';
        }

        // Execute patrol
        async function executePatrol() {
            if (waypoints.length === 0) {
                alert('Please add waypoints first!');
                return;
            }

            const speed = parseFloat(document.getElementById('patrol-speed').value);
            const cycles = parseInt(document.getElementById('patrol-cycles').value);
            const returnToStart = document.getElementById('return-to-start').checked;

            const patrolData = {
                waypoints: waypoints,
                patrol_speed: speed,
                patrol_cycles: cycles,
                return_to_start: returnToStart
            };

            try {
                document.getElementById('path-status').textContent = 'Executing...';
                const response = await fetch(`${API_BASE}/api/robot/patrol`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(patrolData)
                });

                const result = await response.json();
                if (result.success) {
                    addLog(`Patrol started: ${waypoints.length} waypoints, ${cycles} cycle(s)`);
                    document.getElementById('path-status').textContent = 'Active';
                    document.getElementById('path-progress').textContent = '0%';
                } else {
                    addLog(`Patrol failed: ${result.error || 'Unknown error'}`);
                    document.getElementById('path-status').textContent = 'Failed';
                }
            } catch (error) {
                addLog(`Patrol error: ${error.message}`);
                document.getElementById('path-status').textContent = 'Error';
            }
        }

        // Navigation control functions
        async function goToWaypoint() {
            const x = parseFloat(document.getElementById('waypoint-x').value);
            const y = parseFloat(document.getElementById('waypoint-y').value);
            const z = parseFloat(document.getElementById('waypoint-z').value);

            const singleWaypoint = [{ position: { x, y, z } }];

            try {
                const response = await fetch(`${API_BASE}/api/robot/patrol`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        waypoints: singleWaypoint,
                        patrol_speed: parseFloat(document.getElementById('patrol-speed').value),
                        patrol_cycles: 1,
                        return_to_start: false
                    })
                });

                const result = await response.json();
                if (result.success) {
                    addLog(`Going to point (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)})`);
                } else {
                    addLog(`Navigation failed: ${result.error || 'Unknown error'}`);
                }
            } catch (error) {
                addLog(`Navigation error: ${error.message}`);
            }
        }

        async function returnHome() {
            const homeWaypoint = [{ position: { x: 0.0, y: 0.0, z: 0.0 } }];

            try {
                const response = await fetch(`${API_BASE}/api/robot/patrol`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        waypoints: homeWaypoint,
                        patrol_speed: 0.5,
                        patrol_cycles: 1,
                        return_to_start: false
                    })
                });

                const result = await response.json();
                if (result.success) {
                    addLog('Returning to home position (0, 0, 0)');
                } else {
                    addLog(`Return home failed: ${result.error || 'Unknown error'}`);
                }
            } catch (error) {
                addLog(`Return home error: ${error.message}`);
            }
        }

        function pausePatrol() {
            addLog('Pause functionality requires ROS2 action cancellation');
            // This would need ROS2 action server cancellation support
        }

        function stopPatrol() {
            emergencyStop();
            document.getElementById('path-status').textContent = 'Stopped';
            addLog('Patrol stopped (emergency stop triggered)');
        }

        // Save/Load path functions
        function savePath() {
            const pathName = document.getElementById('path-name').value.trim();
            if (!pathName) {
                alert('Please enter a path name');
                return;
            }

            if (waypoints.length === 0) {
                alert('No waypoints to save');
                return;
            }

            savedPaths[pathName] = {
                waypoints: JSON.parse(JSON.stringify(waypoints)),
                created: new Date().toISOString()
            };

            localStorage.setItem('robotPaths', JSON.stringify(savedPaths));
            updateSavedPathsList();
            addLog(`Path "${pathName}" saved with ${waypoints.length} waypoints`);
            document.getElementById('path-name').value = '';
        }

        function loadPath() {
            const pathName = document.getElementById('path-name').value.trim();
            if (!pathName) {
                alert('Please enter a path name to load');
                return;
            }

            if (savedPaths[pathName]) {
                waypoints = JSON.parse(JSON.stringify(savedPaths[pathName].waypoints));
                updateWaypointList();
                updatePathVisualization();
                addLog(`Path "${pathName}" loaded with ${waypoints.length} waypoints`);
            } else {
                alert(`Path "${pathName}" not found`);
            }
        }

        function deleteSavedPath(pathName) {
            delete savedPaths[pathName];
            localStorage.setItem('robotPaths', JSON.stringify(savedPaths));
            updateSavedPathsList();
            addLog(`Path "${pathName}" deleted`);
        }

        function updateSavedPathsList() {
            const listContainer = document.getElementById('saved-paths-list');
            listContainer.innerHTML = '';

            const pathNames = Object.keys(savedPaths);
            if (pathNames.length === 0) {
                listContainer.innerHTML = '<div style="color: #64748b; text-align: center; padding: 10px;">No saved paths</div>';
                return;
            }

            pathNames.forEach(name => {
                const path = savedPaths[name];
                const item = document.createElement('div');
                item.className = 'waypoint-item';
                item.innerHTML = `
                    <div class="waypoint-info">
                        <div style="font-weight: 600;">${name}</div>
                        <div class="waypoint-coords">${path.waypoints.length} waypoints</div>
                    </div>
                    <div class="waypoint-actions">
                        <button onclick="document.getElementById('path-name').value='${name}'; loadPath();" style="color: #3b82f6; margin-right: 8px;">
                            <i class="fas fa-folder-open"></i>
                        </button>
                        <button onclick="deleteSavedPath('${name}')">
                            <i class="fas fa-trash"></i>
                        </button>
                    </div>
                `;
                listContainer.appendChild(item);
            });
        }

        // Initialize path planning when tab is opened
        const originalShowTab = showTab;
        showTab = function(tabName) {
            originalShowTab(tabName);
            if (tabName === 'path-planning') {
                initializeCanvas();
                updateWaypointList();
                updateSavedPathsList();
            }
        };

        // Initialize saved paths list
        updateSavedPathsList();
    </script>
</body>
</html>
"""

class WebRobotInterface(Node):
    def __init__(self):
        super().__init__('web_robot_interface')

        # Initialize SPI for MCP3008 ADC
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1350000
        
        # Sharp GP2Y0A02YK0F sensor channel mapping on MCP3008
        self.sensor_channels = {
            'left_front': 0,
            'left_back': 1,
            'right_front': 2,
            'right_back': 3,
            'back_left': 4,
            'back_right': 5
        }
        
        # ADC reference voltage (typically 3.3V on Raspberry Pi)
        self.adc_vref = 3.3
        self.adc_resolution = 1024

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
        
        @self.app.route('/api/robot/sensors')
        def get_sensors():
            try:
                sensor_data = self.read_all_sensors()
                return jsonify({
                    'success': True,
                    'data': sensor_data,
                    'timestamp': time.time()
                })
            except Exception as e:
                self.get_logger().error(f'Sensor read error: {str(e)}')
                return jsonify({
                    'success': False,
                    'error': str(e),
                    'timestamp': time.time()
                }), 500

        @self.app.route('/api/robot/status')
        def get_status():
            return jsonify({
                'success': True,
                'data': {
                    'mode': 'MANUAL',
                    'emergency_stop': False,
                    'battery_voltage': 24.0,
                    'system_status': 'operational'
                },
                'timestamp': time.time()
            })

        self.get_logger().info('Professional Robot Web Interface initialized')
        self.get_logger().info('Access the interface at: http://localhost:8000 (local) or http://<robot-ip>:8000 (remote)')
        self.get_logger().info('API calls will use the hostname from the browser URL')
    
    def read_adc_channel(self, channel):
        """
        Read analog value from MCP3008 ADC channel (0-7)
        Returns raw ADC value (0-1023)
        """
        if channel < 0 or channel > 7:
            raise ValueError("ADC channel must be between 0 and 7")
        
        # MCP3008 command: start bit + single-ended mode + channel selection
        adc = self.spi.xfer2([1, (8 + channel) << 4, 0])
        data = ((adc[1] & 3) << 8) + adc[2]
        return data
    
    def adc_to_voltage(self, adc_value):
        """
        Convert ADC value to voltage based on reference voltage
        """
        return (adc_value * self.adc_vref) / self.adc_resolution
    
    def sharp_gp2y0a02_voltage_to_distance(self, voltage):
        """
        Convert voltage to distance in mm for Sharp GP2Y0A02YK0F sensor
        Range: 20-150cm (200-1500mm)
        Voltage range: ~2.7V at 20cm to ~0.4V at 150cm
        
        Using empirical formula from datasheet:
        Distance (cm) = 60 * (Voltage ^ -1.1) - 1
        
        Returns distance in mm, or None if out of range
        """
        try:
            # Voltage threshold checks
            if voltage < 0.3:
                return None
            if voltage > 2.8:
                return 200
            
            # Calculate distance using inverse power relationship
            distance_cm = 60 * (voltage ** -1.1) - 1
            
            # Clamp to valid range
            if distance_cm < 20:
                distance_cm = 20
            elif distance_cm > 150:
                distance_cm = 150
            
            # Convert to mm and round
            distance_mm = round(distance_cm * 10)
            return distance_mm
            
        except (ValueError, ZeroDivisionError):
            return None
    
    def read_sharp_sensor(self, channel):
        """
        Read a Sharp GP2Y0A02YK0F sensor and return distance in mm
        """
        try:
            # Take multiple readings and average for stability
            readings = []
            for _ in range(5):
                adc_value = self.read_adc_channel(channel)
                voltage = self.adc_to_voltage(adc_value)
                distance = self.sharp_gp2y0a02_voltage_to_distance(voltage)
                if distance is not None:
                    readings.append(distance)
                time.sleep(0.001)
            
            if readings:
                # Return median to filter out noise
                readings.sort()
                return readings[len(readings) // 2]
            else:
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error reading Sharp sensor on channel {channel}: {str(e)}')
            return None
    
    def read_all_sensors(self):
        """
        Read all sensors and return structured data
        """
        sensor_data = {
            'laser_sensors': {},
            'ultrasonic_sensors': {
                'front_left': None,
                'front_right': None
            },
            'tf_luna': {
                'distance': None,
                'strength': None
            },
            'line_sensors': {
                'left': False,
                'center': False,
                'right': False
            },
            'container_sensors': {
                'left_front': False,
                'left_back': False,
                'right_front': False,
                'right_back': False
            },
            'imu': {
                'yaw': 0.0,
                'pitch': 0.0,
                'roll': 0.0
            }
        }
        
        # Read all Sharp GP2Y0A02YK0F analog distance sensors
        for sensor_name, channel in self.sensor_channels.items():
            distance = self.read_sharp_sensor(channel)
            sensor_data['laser_sensors'][sensor_name] = distance
        
        return sensor_data

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
