"""
New HTML Template for Autonomous Mobile Manipulator Control
Uses Carbon Design System with dark theme, sidebar navigation, and Phosphor icons.
"""

HTML_TEMPLATE = r"""
<!DOCTYPE html>
<html lang="en" data-theme="dark">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Autonomous Mobile Manipulator Control</title>
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/@phosphor-icons/web@2.1.2/src/bold/style.css" />
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <style>
        :root {
            /* Carbon Dark Theme Tokens */
            --cds-bg: #161616;
            --cds-bg-secondary: #262626;
            --cds-bg-tertiary: #393939;
            --cds-layer: #161616;
            --cds-layer-hover: #262626;
            --cds-layer-active: #393939;
            --cds-layer-selected: #0043ce;
            --cds-layer-selected-hover: #0353e9;
            --cds-layer-accelerator: rgba(141, 141, 141, 0.04);
            --cds-border-subtle: #393939;
            --cds-border-strong: #525252;
            --cds-border-interactive: #0f62fe;
            --cds-text-primary: #f4f4f4;
            --cds-text-secondary: #c6c6c6;
            --cds-text-placeholder: #6f6f6f;
            --cds-text-on-color: #ffffff;
            --cds-text-error: #ff8389;
            --cds-support-error: #ff8389;
            --cds-support-success: #42be65;
            --cds-support-warning: #f1c21b;
            --cds-support-info: #4589ff;
            --cds-link: #78a9ff;
            --cds-link-hover: #a6c8ff;
            --cds-button-primary: #0f62fe;
            --cds-button-primary-hover: #0353e9;
            --cds-button-danger: #da1e28;
            --cds-button-danger-hover: #a81d24;
            --cds-button-secondary: #393939;
            --cds-button-secondary-hover: #4c4c4c;
            --cds-tag-background: #393939;
            --cds-tag-text: #c6c6c6;
            --cds-skeleton: #393939;
            --cds-focus: #0f62fe;
            --cds-focus-inset: #161616;
            --cds-notification-error: #fff1f1;
            --cds-notification-success: #defbe6;
            --cds-notification-warning: #fdf6dd;
            --cds-notification-info: #edf5ff;

            /* Carbon Typography */
            --cds-font-family: 'IBM Plex Sans', 'Helvetica Neue', Arial, sans-serif;
            --cds-font-family-mono: 'IBM Plex Mono', 'Menlo', monospace;
            --cds-font-size-12: 0.75rem;
            --cds-font-size-14: 0.875rem;
            --cds-font-size-16: 1rem;
            --cds-font-size-20: 1.25rem;
            --cds-font-size-24: 1.5rem;
            --cds-font-size-28: 1.75rem;

            /* Carbon Spacing */
            --cds-spacing-01: 2px;
            --cds-spacing-02: 4px;
            --cds-spacing-03: 8px;
            --cds-spacing-04: 12px;
            --cds-spacing-05: 16px;
            --cds-spacing-06: 24px;
            --cds-spacing-07: 32px;
            --cds-spacing-08: 40px;
            --cds-spacing-09: 48px;
            --cds-spacing-10: 64px;

            /* Carbon Elevation */
            --cds-elevation-01: 0 1px 2px rgba(0,0,0,0.3);
            --cds-elevation-02: 0 2px 6px rgba(0,0,0,0.3);
            --cds-elevation-03: 0 4px 8px rgba(0,0,0,0.3);
            --cds-elevation-04: 0 8px 16px rgba(0,0,0,0.3);

            /* Layout */
            --sidebar-width: 240px;
            --sidebar-collapsed: 64px;
            --header-height: 48px;
        }

        *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

        body {
            font-family: var(--cds-font-family);
            font-size: var(--cds-font-size-14);
            line-height: 1.4;
            background: var(--cds-bg);
            color: var(--cds-text-primary);
            overflow: hidden;
            height: 100vh;
        }

        /* Layout */
        .app-layout {
            display: flex;
            height: 100vh;
        }

        /* Sidebar */
        .sidebar {
            width: var(--sidebar-width);
            background: var(--cds-bg);
            border-right: 1px solid var(--cds-border-subtle);
            display: flex;
            flex-direction: column;
            flex-shrink: 0;
            transition: width 0.2s ease;
            z-index: 100;
        }

        .sidebar-header {
            height: 48px;
            display: flex;
            align-items: center;
            padding: 0 var(--cds-spacing-05);
            border-bottom: 1px solid var(--cds-border-subtle);
            gap: var(--cds-spacing-03);
        }

        .sidebar-logo {
            font-size: var(--cds-font-size-16);
            font-weight: 600;
            color: var(--cds-text-primary);
            white-space: nowrap;
            overflow: hidden;
        }

        .sidebar-logo i { font-size: 20px; color: var(--cds-support-info); }

        .sidebar-nav {
            flex: 1;
            overflow-y: auto;
            padding: var(--cds-spacing-03) 0;
        }

        .nav-section {
            padding: var(--cds-spacing-02) var(--cds-spacing-05);
            font-size: var(--cds-font-size-12);
            font-weight: 600;
            color: var(--cds-text-placeholder);
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        .nav-item {
            display: flex;
            align-items: center;
            gap: var(--cds-spacing-03);
            padding: var(--cds-spacing-03) var(--cds-spacing-05);
            color: var(--cds-text-secondary);
            cursor: pointer;
            transition: all 0.15s ease;
            border-left: 3px solid transparent;
            font-size: var(--cds-font-size-14);
            text-decoration: none;
        }

        .nav-item:hover {
            background: var(--cds-layer-hover);
            color: var(--cds-text-primary);
        }

        .nav-item.active {
            background: var(--cds-layer-selected);
            color: var(--cds-text-on-color);
            border-left-color: var(--cds-support-info);
        }

        .nav-item i { font-size: 18px; width: 20px; text-align: center; }

        .sidebar-footer {
            padding: var(--cds-spacing-03) var(--cds-spacing-05);
            border-top: 1px solid var(--cds-border-subtle);
        }

        /* Main Content */
        .main-content {
            flex: 1;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }

        /* Header */
        .header {
            height: var(--header-height);
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 0 var(--cds-spacing-06);
            border-bottom: 1px solid var(--cds-border-subtle);
            background: var(--cds-bg);
            flex-shrink: 0;
        }

        .header-left {
            display: flex;
            align-items: center;
            gap: var(--cds-spacing-04);
        }

        .header-title {
            font-size: var(--cds-font-size-16);
            font-weight: 600;
        }

        .header-right {
            display: flex;
            align-items: center;
            gap: var(--cds-spacing-04);
        }

        .header-status {
            display: flex;
            align-items: center;
            gap: var(--cds-spacing-02);
            font-size: var(--cds-font-size-12);
            color: var(--cds-text-secondary);
        }

        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: var(--cds-text-placeholder);
        }

        .status-dot.connected { background: var(--cds-support-success); }
        .status-dot.error { background: var(--cds-support-error); }
        .status-dot.warning { background: var(--cds-support-warning); }

        /* Content Area */
        .content-area {
            flex: 1;
            overflow-y: auto;
            padding: var(--cds-spacing-06);
        }

        .page-section {
            display: none;
        }

        .page-section.active {
            display: block;
        }

        /* Cards */
        .cds-card {
            background: var(--cds-bg-secondary);
            border: 1px solid var(--cds-border-subtle);
            padding: var(--cds-spacing-05);
            margin-bottom: var(--cds-spacing-05);
        }

        .cds-card-header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            margin-bottom: var(--cds-spacing-04);
        }

        .cds-card-title {
            font-size: var(--cds-font-size-14);
            font-weight: 600;
            display: flex;
            align-items: center;
            gap: var(--cds-spacing-02);
        }

        .cds-card-title i { font-size: 16px; color: var(--cds-support-info); }

        /* Buttons */
        .cds-btn {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            gap: var(--cds-spacing-02);
            padding: var(--cds-spacing-03) var(--cds-spacing-05);
            font-family: var(--cds-font-family);
            font-size: var(--cds-font-size-14);
            font-weight: 400;
            border: none;
            cursor: pointer;
            transition: background 0.15s ease, border-color 0.15s ease;
            min-height: 32px;
        }

        .cds-btn:focus-visible {
            outline: 2px solid var(--cds-focus);
            outline-offset: -2px;
        }

        .cds-btn--primary { background: var(--cds-button-primary); color: var(--cds-text-on-color); }
        .cds-btn--primary:hover { background: var(--cds-button-primary-hover); }
        .cds-btn--danger { background: var(--cds-button-danger); color: var(--cds-text-on-color); }
        .cds-btn--danger:hover { background: var(--cds-button-danger-hover); }
        .cds-btn--secondary { background: var(--cds-button-secondary); color: var(--cds-text-primary); }
        .cds-btn--secondary:hover { background: var(--cds-button-secondary-hover); }
        .cds-btn--ghost { background: transparent; color: var(--cds-link); }
        .cds-btn--ghost:hover { background: var(--cds-layer-hover); }
        .cds-btn--sm { padding: var(--cds-spacing-02) var(--cds-spacing-03); min-height: 24px; font-size: var(--cds-font-size-12); }
        .cds-btn--lg { padding: var(--cds-spacing-04) var(--cds-spacing-06); min-height: 40px; }
        .cds-btn:disabled { opacity: 0.5; cursor: not-allowed; }

        /* Inputs */
        .cds-input {
            width: 100%;
            padding: var(--cds-spacing-03) var(--cds-spacing-04);
            background: var(--cds-bg-tertiary);
            border: 1px solid var(--cds-border-strong);
            color: var(--cds-text-primary);
            font-family: var(--cds-font-family);
            font-size: var(--cds-font-size-14);
            min-height: 32px;
        }

        .cds-input:focus {
            outline: none;
            border-color: var(--cds-border-interactive);
            box-shadow: 0 0 0 1px var(--cds-border-interactive);
        }

        .cds-input::placeholder { color: var(--cds-text-placeholder); }

        .cds-select {
            appearance: none;
            background-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'%3E%3Cpath fill='%23c6c6c6' d='M16 22L6 12h20z'/%3E%3C/svg%3E");
            background-repeat: no-repeat;
            background-position: right 8px center;
            background-size: 12px;
            padding-right: 32px;
        }

        .cds-label {
            display: block;
            font-size: var(--cds-font-size-12);
            font-weight: 600;
            color: var(--cds-text-secondary);
            margin-bottom: var(--cds-spacing-02);
        }

        .cds-form-group {
            margin-bottom: var(--cds-spacing-05);
        }

        /* Grid */
        .cds-grid {
            display: grid;
            gap: var(--cds-spacing-05);
        }

        .cds-grid--2 { grid-template-columns: repeat(2, 1fr); }
        .cds-grid--3 { grid-template-columns: repeat(3, 1fr); }
        .cds-grid--4 { grid-template-columns: repeat(4, 1fr); }
        .cds-grid--auto { grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); }

        /* Tags */
        .cds-tag {
            display: inline-flex;
            align-items: center;
            gap: var(--cds-spacing-02);
            padding: var(--cds-spacing-01) var(--cds-spacing-03);
            background: var(--cds-tag-background);
            color: var(--cds-tag-text);
            font-size: var(--cds-font-size-12);
        }

        .cds-tag--success { background: #1e4d2b; color: #a7f0ba; }
        .cds-tag--error { background: #4e1e1e; color: #ffc2c5; }
        .cds-tag--warning { background: #4a3800; color: #f8d44d; }
        .cds-tag--info { background: #002d6c; color: #a6c8ff; }

        /* Toggle */
        .cds-toggle {
            position: relative;
            display: inline-block;
            width: 48px;
            height: 24px;
        }

        .cds-toggle input {
            opacity: 0;
            width: 0;
            height: 0;
        }

        .cds-toggle-slider {
            position: absolute;
            cursor: pointer;
            top: 0; left: 0; right: 0; bottom: 0;
            background: var(--cds-button-secondary);
            border: 1px solid var(--cds-border-strong);
            transition: 0.2s;
        }

        .cds-toggle-slider::before {
            content: '';
            position: absolute;
            width: 16px;
            height: 16px;
            left: 3px;
            bottom: 3px;
            background: var(--cds-text-primary);
            transition: 0.2s;
        }

        .cds-toggle input:checked + .cds-toggle-slider { background: var(--cds-support-success); border-color: var(--cds-support-success); }
        .cds-toggle input:checked + .cds-toggle-slider::before { transform: translateX(24px); }
        .cds-toggle input:focus-visible + .cds-toggle-slider { outline: 2px solid var(--cds-focus); outline-offset: 2px; }

        /* Table */
        .cds-table {
            width: 100%;
            border-collapse: collapse;
            font-size: var(--cds-font-size-14);
        }

        .cds-table th {
            text-align: left;
            padding: var(--cds-spacing-03) var(--cds-spacing-04);
            background: var(--cds-bg-tertiary);
            color: var(--cds-text-secondary);
            font-weight: 600;
            border-bottom: 1px solid var(--cds-border-subtle);
        }

        .cds-table td {
            padding: var(--cds-spacing-03) var(--cds-spacing-04);
            border-bottom: 1px solid var(--cds-border-subtle);
        }

        .cds-table tr:hover td { background: var(--cds-layer-hover); }

        /* Sensor Grid */
        .sensor-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(160px, 1fr));
            gap: var(--cds-spacing-04);
        }

        .sensor-card {
            background: var(--cds-bg-tertiary);
            border: 1px solid var(--cds-border-subtle);
            padding: var(--cds-spacing-04);
            text-align: center;
        }

        .sensor-value {
            font-size: var(--cds-font-size-24);
            font-weight: 600;
            font-family: var(--cds-font-family-mono);
            color: var(--cds-support-info);
            margin: var(--cds-spacing-02) 0;
        }

        .sensor-label {
            font-size: var(--cds-font-size-12);
            color: var(--cds-text-secondary);
        }

        /* D-Pad */
        .dpad {
            display: grid;
            grid-template-columns: repeat(3, 56px);
            grid-template-rows: repeat(3, 56px);
            gap: 4px;
            justify-content: center;
        }

        .dpad-btn {
            display: flex;
            align-items: center;
            justify-content: center;
            background: var(--cds-bg-tertiary);
            border: 1px solid var(--cds-border-strong);
            color: var(--cds-text-primary);
            cursor: pointer;
            font-size: 18px;
            transition: all 0.1s;
        }

        .dpad-btn:hover { background: var(--cds-layer-active); }
        .dpad-btn:active { transform: scale(0.95); }
        .dpad-btn--center { background: var(--cds-button-danger); color: var(--cds-text-on-color); }
        .dpad-btn--center:hover { background: var(--cds-button-danger-hover); }

        /* KPI Strip */
        .kpi-strip {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
            gap: var(--cds-spacing-04);
            margin-bottom: var(--cds-spacing-06);
        }

        .kpi-card {
            background: var(--cds-bg-secondary);
            border: 1px solid var(--cds-border-subtle);
            padding: var(--cds-spacing-04) var(--cds-spacing-05);
        }

        .kpi-label {
            font-size: var(--cds-font-size-12);
            color: var(--cds-text-secondary);
            margin-bottom: var(--cds-spacing-01);
        }

        .kpi-value {
            font-size: var(--cds-font-size-24);
            font-weight: 600;
            font-family: var(--cds-font-family-mono);
        }

        .kpi-value--ok { color: var(--cds-support-success); }
        .kpi-value--warn { color: var(--cds-support-warning); }
        .kpi-value--error { color: var(--cds-support-error); }
        .kpi-value--info { color: var(--cds-support-info); }

        /* Log */
        .log-output {
            background: var(--cds-bg);
            border: 1px solid var(--cds-border-subtle);
            padding: var(--cds-spacing-04);
            font-family: var(--cds-font-family-mono);
            font-size: var(--cds-font-size-12);
            max-height: 300px;
            overflow-y: auto;
            line-height: 1.6;
        }

        /* Modal */
        .cds-modal-overlay {
            display: none;
            position: fixed;
            top: 0; left: 0; right: 0; bottom: 0;
            background: rgba(0,0,0,0.6);
            z-index: 1000;
            align-items: center;
            justify-content: center;
        }

        .cds-modal-overlay.open { display: flex; }

        .cds-modal {
            background: var(--cds-bg-secondary);
            border: 1px solid var(--cds-border-subtle);
            width: 90%;
            max-width: 720px;
            max-height: 85vh;
            overflow-y: auto;
        }

        .cds-modal-header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: var(--cds-spacing-05);
            border-bottom: 1px solid var(--cds-border-subtle);
        }

        .cds-modal-title {
            font-size: var(--cds-font-size-16);
            font-weight: 600;
        }

        .cds-modal-close {
            background: none;
            border: none;
            color: var(--cds-text-secondary);
            cursor: pointer;
            font-size: 20px;
            padding: var(--cds-spacing-02);
        }

        .cds-modal-body { padding: var(--cds-spacing-05); }
        .cds-modal-footer {
            display: flex;
            justify-content: flex-end;
            gap: var(--cds-spacing-03);
            padding: var(--cds-spacing-04) var(--cds-spacing-05);
            border-top: 1px solid var(--cds-border-subtle);
        }

        /* Utilities */
        .hidden { display: none !important; }
        .mono { font-family: var(--cds-font-family-mono); }
        .text-secondary { color: var(--cds-text-secondary); }
        .text-sm { font-size: var(--cds-font-size-12); }
        .mt-4 { margin-top: var(--cds-spacing-04); }
        .mb-4 { margin-bottom: var(--cds-spacing-04); }
        .mb-6 { margin-bottom: var(--cds-spacing-06); }
        .gap-2 { gap: var(--cds-spacing-02); }
        .gap-3 { gap: var(--cds-spacing-03); }
        .flex { display: flex; }
        .items-center { align-items: center; }
        .justify-between { justify-content: space-between; }
        .flex-wrap { flex-wrap: wrap; }

        /* Scrollbar */
        ::-webkit-scrollbar { width: 6px; }
        ::-webkit-scrollbar-track { background: var(--cds-bg); }
        ::-webkit-scrollbar-thumb { background: var(--cds-bg-tertiary); }
        ::-webkit-scrollbar-thumb:hover { background: var(--cds-border-strong); }

        /* Responsive */
        @media (max-width: 1024px) {
            .cds-grid--2, .cds-grid--3, .cds-grid--4 { grid-template-columns: 1fr; }
            .kpi-strip { grid-template-columns: repeat(2, 1fr); }
        }

        @media (max-width: 768px) {
            .sidebar { width: var(--sidebar-collapsed); }
            .sidebar-logo span, .nav-item span, .nav-section { display: none; }
            .nav-item { justify-content: center; padding: var(--cds-spacing-03); }
            .nav-item i { margin: 0; }
        }
    </style>
</head>
<body>
    <div class="app-layout">
        <!-- Sidebar -->
        <aside class="sidebar">
            <div class="sidebar-header">
                <i class="ph-bold ph-robot"></i>
                <span class="sidebar-logo">Robot Control</span>
            </div>
            <nav class="sidebar-nav">
                <div class="nav-section">Overview</div>
                <a class="nav-item active" onclick="navigateTo('dashboard')" data-page="dashboard">
                    <i class="ph-bold ph-squares-four"></i>
                    <span>Dashboard</span>
                </a>
                <a class="nav-item" onclick="navigateTo('sensors')" data-page="sensors">
                    <i class="ph-bold ph-activity"></i>
                    <span>Sensors</span>
                </a>

                <div class="nav-section">Control</div>
                <a class="nav-item" onclick="navigateTo('movement')" data-page="movement">
                    <i class="ph-bold ph-compass"></i>
                    <span>Movement</span>
                </a>
                <a class="nav-item" onclick="navigateTo('manipulation')" data-page="manipulation">
                    <i class="ph-bold ph-hand-grabbing"></i>
                    <span>Manipulation</span>
                </a>
                <a class="nav-item" onclick="navigateTo('pathplanning')" data-page="pathplanning">
                    <i class="ph-bold ph-map-trifold"></i>
                    <span>Path Planning</span>
                </a>

                <div class="nav-section">Automation</div>
                <a class="nav-item" onclick="navigateTo('automation')" data-page="automation">
                    <i class="ph-bold ph-lightning"></i>
                    <span>Rules</span>
                </a>

                <div class="nav-section">System</div>
                <a class="nav-item" onclick="navigateTo('serial')" data-page="serial">
                    <i class="ph-bold ph-terminal"></i>
                    <span>Serial</span>
                </a>
                <a class="nav-item" onclick="navigateTo('serialmonitor')" data-page="serialmonitor">
                    <i class="ph-bold ph-monitor"></i>
                    <span>Monitor</span>
                </a>
            </nav>
            <div class="sidebar-footer">
                <div class="header-status">
                    <div id="mega-dot" class="status-dot"></div>
                    <span id="mega-label">Mega: --</span>
                </div>
            </div>
        </aside>

        <!-- Main -->
        <div class="main-content">
            <header class="header">
                <div class="header-left">
                    <span class="header-title" id="page-title">Dashboard</span>
                </div>
                <div class="header-right">
                    <span class="header-status" id="ros2-status-header">
                        <div class="status-dot" id="ros2-dot"></div>
                        <span>ROS2: --</span>
                    </span>
                    <span class="header-status">
                        <span class="mono text-sm" id="header-time">--:--</span>
                    </span>
                </div>
            </header>

            <div class="content-area">
                <!-- DASHBOARD -->
                <section id="page-dashboard" class="page-section active">
                    <div class="kpi-strip">
                        <div class="kpi-card">
                            <div class="kpi-label">Status</div>
                            <div class="kpi-value kpi-value--info" id="kpi-status">--</div>
                        </div>
                        <div class="kpi-card">
                            <div class="kpi-label">Mega</div>
                            <div class="kpi-value" id="kpi-mega">--</div>
                        </div>
                        <div class="kpi-card">
                            <div class="kpi-label">Heading</div>
                            <div class="kpi-value kpi-value--info mono" id="kpi-heading">--</div>
                        </div>
                        <div class="kpi-card">
                            <div class="kpi-label">Position</div>
                            <div class="kpi-value mono text-sm" id="kpi-position">--</div>
                        </div>
                        <div class="kpi-card">
                            <div class="kpi-label">Front Left</div>
                            <div class="kpi-value mono" id="kpi-lf">--</div>
                        </div>
                        <div class="kpi-card">
                            <div class="kpi-label">Front Right</div>
                            <div class="kpi-value mono" id="kpi-rf">--</div>
                        </div>
                    </div>

                    <div class="cds-grid cds-grid--2">
                        <div class="cds-card">
                            <div class="cds-card-header">
                                <span class="cds-card-title"><i class="ph-bold ph-lightning"></i> Quick Actions</span>
                            </div>
                            <div class="cds-grid cds-grid--4">
                                <button class="cds-btn cds-btn--danger cds-btn--lg" onclick="emergencyStop()" style="grid-column: span 4;">
                                    <i class="ph-bold ph-warning"></i> Emergency Stop
                                </button>
                                <button class="cds-btn cds-btn--secondary" onclick="homeServos()"><i class="ph-bold ph-house"></i> Home Servos</button>
                                <button class="cds-btn cds-btn--secondary" onclick="testLimitSwitches()"><i class="ph-bold ph-magnifying-glass"></i> Test Limits</button>
                                <button class="cds-btn cds-btn--secondary" onclick="refreshSensors()"><i class="ph-bold ph-arrows-clockwise"></i> Refresh</button>
                                <button class="cds-btn cds-btn--secondary" onclick="reconnectMega()"><i class="ph-bold ph-plugs"></i> Reconnect</button>
                            </div>
                        </div>

                        <div class="cds-card">
                            <div class="cds-card-header">
                                <span class="cds-card-title"><i class="ph-bold ph-notebook"></i> Event Log</span>
                                <button class="cds-btn cds-btn--ghost cds-btn--sm" onclick="clearLog()">Clear</button>
                            </div>
                            <div id="robot-log" class="log-output">System initialized...</div>
                        </div>
                    </div>
                </section>

                <!-- SENSORS -->
                <section id="page-sensors" class="page-section">
                    <div class="cds-card">
                        <div class="cds-card-header">
                            <span class="cds-card-title"><i class="ph-bold ph-eye"></i> IR Distance Sensors</span>
                            <span class="cds-tag cds-tag--info">6 sensors</span>
                        </div>
                        <div class="sensor-grid" id="ir-sensors">
                            <div class="sensor-card"><div class="sensor-label">Left Front</div><div class="sensor-value" id="s-lf">--</div><div class="sensor-label">cm</div></div>
                            <div class="sensor-card"><div class="sensor-label">Left Back</div><div class="sensor-value" id="s-lb">--</div><div class="sensor-label">cm</div></div>
                            <div class="sensor-card"><div class="sensor-label">Right Front</div><div class="sensor-value" id="s-rf">--</div><div class="sensor-label">cm</div></div>
                            <div class="sensor-card"><div class="sensor-label">Right Back</div><div class="sensor-value" id="s-rb">--</div><div class="sensor-label">cm</div></div>
                            <div class="sensor-card"><div class="sensor-label">Back Left</div><div class="sensor-value" id="s-bl">--</div><div class="sensor-label">cm</div></div>
                            <div class="sensor-card"><div class="sensor-label">Back Right</div><div class="sensor-value" id="s-br">--</div><div class="sensor-label">cm</div></div>
                        </div>
                    </div>

                    <div class="cds-grid cds-grid--2">
                        <div class="cds-card">
                            <div class="cds-card-header">
                                <span class="cds-card-title"><i class="ph-bold ph-radar"></i> Ultrasonic Sensors</span>
                            </div>
                            <div class="sensor-grid">
                                <div class="sensor-card"><div class="sensor-label">Front Left</div><div class="sensor-value" id="s-ultra-fl">--</div><div class="sensor-label">cm</div></div>
                                <div class="sensor-card"><div class="sensor-label">Front Right</div><div class="sensor-value" id="s-ultra-fr">--</div><div class="sensor-label">cm</div></div>
                            </div>
                        </div>

                        <div class="cds-card">
                            <div class="cds-card-header">
                                <span class="cds-card-title"><i class="ph-bold ph-compass"></i> IMU</span>
                            </div>
                            <div class="sensor-grid">
                                <div class="sensor-card"><div class="sensor-label">Heading</div><div class="sensor-value" id="s-imu-h">--</div><div class="sensor-label">deg</div></div>
                                <div class="sensor-card"><div class="sensor-label">Pitch</div><div class="sensor-value" id="s-imu-p">--</div><div class="sensor-label">deg</div></div>
                                <div class="sensor-card"><div class="sensor-label">Roll</div><div class="sensor-value" id="s-imu-r">--</div><div class="sensor-label">deg</div></div>
                            </div>
                        </div>
                    </div>

                    <div class="cds-card">
                        <div class="cds-card-header">
                            <span class="cds-card-title"><i class="ph-bold ph-lines-steeper"></i> Line Sensors</span>
                        </div>
                        <div class="sensor-grid">
                            <div class="sensor-card"><div class="sensor-label">Left</div><div class="sensor-value" id="s-line-l">--</div></div>
                            <div class="sensor-card"><div class="sensor-label">Center</div><div class="sensor-value" id="s-line-c">--</div></div>
                            <div class="sensor-card"><div class="sensor-label">Right</div><div class="sensor-value" id="s-line-r">--</div></div>
                        </div>
                    </div>
                </section>

                <!-- MOVEMENT -->
                <section id="page-movement" class="page-section">
                    <div class="cds-grid cds-grid--2">
                        <div class="cds-card">
                            <div class="cds-card-header">
                                <span class="cds-card-title"><i class="ph-bold ph-game-controller"></i> Movement</span>
                            </div>
                            <div class="dpad">
                                <div></div>
                                <button class="dpad-btn" onclick="moveRobot('forward')"><i class="ph-bold ph-caret-up"></i></button>
                                <div></div>
                                <button class="dpad-btn" onclick="turnRobot('left')"><i class="ph-bold ph-caret-left"></i></button>
                                <button class="dpad-btn dpad-btn--center" onclick="stopRobot()"><i class="ph-bold ph-square"></i></button>
                                <button class="dpad-btn" onclick="turnRobot('right')"><i class="ph-bold ph-caret-right"></i></button>
                                <div></div>
                                <button class="dpad-btn" onclick="moveRobot('backward')"><i class="ph-bold ph-caret-down"></i></button>
                                <div></div>
                            </div>
                        </div>

                        <div class="cds-card">
                            <div class="cds-card-header">
                                <span class="cds-card-title"><i class="ph-bold ph-gauge"></i> Speed</span>
                            </div>
                            <div class="cds-form-group">
                                <input type="range" id="speed-slider" min="0" max="100" value="50" oninput="updateSpeedDisplay()" style="width:100%">
                                <div class="flex justify-between mt-4">
                                    <span class="text-sm text-secondary">0%</span>
                                    <span class="mono" id="speed-display" style="font-size:var(--cds-font-size-20);font-weight:600;color:var(--cds-support-info)">50%</span>
                                    <span class="text-sm text-secondary">100%</span>
                                </div>
                            </div>
                            <button class="cds-btn cds-btn--primary cds-btn--lg" onclick="setSpeed()" style="width:100%">Apply Speed</button>
                        </div>
                    </div>

                    <div class="cds-card">
                        <div class="cds-card-header">
                            <span class="cds-card-title"><i class="ph-bold ph-wheel"></i> Presets</span>
                        </div>
                        <div class="flex flex-wrap gap-2">
                            <button class="cds-btn cds-btn--secondary" onclick="executePreset('square')">Square</button>
                            <button class="cds-btn cds-btn--secondary" onclick="executePreset('figure8')">Figure-8</button>
                            <button class="cds-btn cds-btn--secondary" onclick="executePreset('lawnmower')">Lawnmower</button>
                            <button class="cds-btn cds-btn--secondary" onclick="executePreset('scan')">Scan</button>
                        </div>
                    </div>
                </section>

                <!-- MANIPULATION -->
                <section id="page-manipulation" class="page-section">
                    <div class="cds-grid cds-grid--2">
                        <div class="cds-card">
                            <div class="cds-card-header">
                                <span class="cds-card-title"><i class="ph-bold ph-hand-grabbing"></i> Gripper</span>
                            </div>
                            <div class="flex flex-wrap gap-2 mb-4">
                                <button class="cds-btn cds-btn--primary" onclick="controlGripper('open_full')">Open Full</button>
                                <button class="cds-btn cds-btn--primary" onclick="controlGripper('open_half')">Open Half</button>
                                <button class="cds-btn cds-btn--danger" onclick="controlGripper('close')">Close</button>
                            </div>
                            <div class="cds-form-group">
                                <label class="cds-label">Tilt Angle</label>
                                <input type="range" id="gripper-tilt" min="0" max="180" value="90" oninput="updateGripperTiltDisplay()" style="width:100%">
                                <div class="flex justify-between mt-4">
                                    <span class="text-sm text-secondary">0</span>
                                    <span class="mono" id="gripper-tilt-display" style="font-size:var(--cds-font-size-20);font-weight:600;color:var(--cds-support-info)">90&deg;</span>
                                    <span class="text-sm text-secondary">180</span>
                                </div>
                            </div>
                            <button class="cds-btn cds-btn--primary" onclick="setGripperTilt()" style="width:100%">Set Tilt</button>
                        </div>

                        <div class="cds-card">
                            <div class="cds-card-header">
                                <span class="cds-card-title"><i class="ph-bold ph-package"></i> Container</span>
                            </div>
                            <div class="cds-form-group">
                                <label class="cds-label">Container ID</label>
                                <select id="container-id" class="cds-input cds-select">
                                    <option value="1">Container 1</option>
                                    <option value="2">Container 2</option>
                                    <option value="3">Container 3</option>
                                </select>
                            </div>
                            <div class="flex flex-wrap gap-2">
                                <button class="cds-btn cds-btn--primary" onclick="controlContainer('open')">Open</button>
                                <button class="cds-btn cds-btn--danger" onclick="controlContainer('close')">Close</button>
                                <button class="cds-btn cds-btn--secondary" onclick="controlContainer('lock')">Lock</button>
                                <button class="cds-btn cds-btn--secondary" onclick="controlContainer('unlock')">Unlock</button>
                            </div>
                        </div>
                    </div>
                </section>

                <!-- PATH PLANNING -->
                <section id="page-pathplanning" class="page-section">
                    <div class="cds-card">
                        <div class="cds-card-header">
                            <span class="cds-card-title"><i class="ph-bold ph-route"></i> Movement Sets</span>
                        </div>
                        <div class="flex flex-wrap gap-2">
                            <button class="cds-btn cds-btn--primary" onclick="executeMovementSet('lawnmower')">Lawn Mower</button>
                            <button class="cds-btn cds-btn--primary" onclick="executeMovementSet('spiral')">Spiral</button>
                            <button class="cds-btn cds-btn--primary" onclick="executeMovementSet('boundary')">Boundary</button>
                            <button class="cds-btn cds-btn--primary" onclick="executeMovementSet('zigzag')">Zigzag</button>
                            <button class="cds-btn cds-btn--primary" onclick="executeMovementSet('figure8')">Figure-8</button>
                            <button class="cds-btn cds-btn--primary" onclick="executeMovementSet('circle')">Circle</button>
                            <button class="cds-btn cds-btn--secondary" onclick="executeMovementSet('return_home')">Return Home</button>
                            <button class="cds-btn cds-btn--secondary" onclick="executeMovementSet('object_search')">Object Search</button>
                            <button class="cds-btn cds-btn--danger" onclick="stopSequence()">Stop</button>
                        </div>
                    </div>

                    <div class="cds-grid cds-grid--2">
                        <div class="cds-card">
                            <div class="cds-card-header">
                                <span class="cds-card-title"><i class="ph-bold ph-map-pin"></i> Waypoints</span>
                            </div>
                            <div class="cds-form-group">
                                <label class="cds-label">Add Waypoint</label>
                                <div class="flex gap-2 mb-4">
                                    <input type="number" id="waypoint-x" step="0.1" placeholder="X (m)" class="cds-input" style="flex:1">
                                    <input type="number" id="waypoint-y" step="0.1" placeholder="Y (m)" class="cds-input" style="flex:1">
                                    <input type="number" id="waypoint-z" step="0.1" placeholder="Z" value="0" class="cds-input" style="flex:1">
                                    <button class="cds-btn cds-btn--primary" onclick="addWaypoint()">Add</button>
                                </div>
                            </div>
                            <div id="waypoint-list" class="log-output" style="min-height:80px;margin-bottom:var(--cds-spacing-04)">No waypoints</div>
                            <div class="flex flex-wrap gap-2">
                                <button class="cds-btn cds-btn--primary" onclick="navigateWaypoints()">Navigate</button>
                                <button class="cds-btn cds-btn--secondary" onclick="previewWaypoints()">Preview</button>
                                <button class="cds-btn cds-btn--danger" onclick="clearWaypoints()">Clear</button>
                                <button class="cds-btn cds-btn--ghost" onclick="optimizeWaypoints()">Optimize</button>
                            </div>
                        </div>

                        <div class="cds-card">
                            <div class="cds-card-header">
                                <span class="cds-card-title"><i class="ph-bold ph-map"></i> Map</span>
                                <button class="cds-btn cds-btn--ghost cds-btn--sm" onclick="loadMap()"><i class="ph-bold ph-arrows-clockwise"></i></button>
                            </div>
                            <div id="map-container" style="background:var(--cds-bg);border:1px solid var(--cds-border-subtle);min-height:300px;display:flex;align-items:center;justify-content:center;">
                                <span class="text-secondary text-sm">Loading map...</span>
                            </div>
                        </div>
                    </div>
                </section>

                <!-- AUTOMATION -->
                <section id="page-automation" class="page-section">
                    <div class="cds-card">
                        <div class="cds-card-header">
                            <span class="cds-card-title"><i class="ph-bold ph-lightning"></i> Automation Rules</span>
                            <div class="flex gap-2">
                                <button class="cds-btn cds-btn--primary" onclick="createAutomation()"><i class="ph-bold ph-plus"></i> New Rule</button>
                                <button class="cds-btn cds-btn--ghost" onclick="refreshAutomations()"><i class="ph-bold ph-arrows-clockwise"></i></button>
                            </div>
                        </div>
                        <div id="automation-list">
                            <div style="text-align:center;padding:var(--cds-spacing-08);color:var(--cds-text-placeholder)">
                                <i class="ph-bold ph-lightning" style="font-size:48px;display:block;margin-bottom:var(--cds-spacing-03)"></i>
                                Loading automations...
                            </div>
                        </div>
                    </div>
                </section>

                <!-- SERIAL -->
                <section id="page-serial" class="page-section">
                    <div class="cds-card">
                        <div class="cds-card-header">
                            <span class="cds-card-title"><i class="ph-bold ph-terminal"></i> Serial Command</span>
                        </div>
                        <div class="flex gap-2 mb-4">
                            <input type="text" id="serial-command" class="cds-input" placeholder="Enter command..." style="flex:1" onkeypress="if(event.key==='Enter')sendSerialCommand()">
                            <button class="cds-btn cds-btn--primary" onclick="sendSerialCommand()">Send</button>
                        </div>
                        <div class="cds-card-header">
                            <span class="cds-card-title"><i class="ph-bold ph-list"></i> Quick Commands</span>
                        </div>
                        <div class="flex flex-wrap gap-2 mb-4">
                            <button class="cds-btn cds-btn--secondary" onclick="sendQuickCommand('f')">Forward</button>
                            <button class="cds-btn cds-btn--secondary" onclick="sendQuickCommand('b')">Backward</button>
                            <button class="cds-btn cds-btn--secondary" onclick="sendQuickCommand('l')">Left</button>
                            <button class="cds-btn cds-btn--secondary" onclick="sendQuickCommand('r')">Right</button>
                            <button class="cds-btn cds-btn--danger" onclick="sendQuickCommand('s')">Stop</button>
                            <button class="cds-btn cds-btn--secondary" onclick="sendQuickCommand('no')">Open</button>
                            <button class="cds-btn cds-btn--secondary" onclick="sendQuickCommand('nc')">Close</button>
                            <button class="cds-btn cds-btn--secondary" onclick="sendQuickCommand('p')">Status</button>
                        </div>
                        <div class="cds-card-header">
                            <span class="cds-card-title"><i class="ph-bold ph-scroll"></i> Serial Log</span>
                        </div>
                        <div id="serial-log" class="log-output" style="min-height:200px">Waiting for commands...</div>
                    </div>
                </section>

                <!-- SERIAL MONITOR -->
                <section id="page-serialmonitor" class="page-section">
                    <div class="cds-card">
                        <div class="cds-card-header">
                            <span class="cds-card-title"><i class="ph-bold ph-monitor"></i> Serial Monitor</span>
                            <div class="flex gap-2">
                                <button class="cds-btn cds-btn--primary" onclick="startSerialMonitor()" id="monitor-start-btn">Start</button>
                                <button class="cds-btn cds-btn--danger hidden" onclick="stopSerialMonitor()" id="monitor-stop-btn">Stop</button>
                                <button class="cds-btn cds-btn--ghost" onclick="clearSerialMonitor()">Clear</button>
                            </div>
                        </div>
                        <div class="flex gap-3 mb-4 items-center">
                            <div class="flex items-center gap-2">
                                <label class="cds-label" style="margin:0">Auto-scroll</label>
                                <input type="checkbox" id="monitor-autoscroll" checked>
                            </div>
                            <div class="flex items-center gap-2">
                                <div id="monitor-connection-status" class="status-dot"></div>
                                <span id="monitor-connection-text" class="text-sm text-secondary">Disconnected</span>
                            </div>
                            <span class="text-sm text-secondary" id="data-rate-badge">0 B/s</span>
                        </div>
                        <div id="serial-monitor-output" class="log-output" style="min-height:400px">
                            <div style="text-align:center;padding:var(--cds-spacing-08);color:var(--cds-text-placeholder)">
                                Click "Start" to begin monitoring
                            </div>
                        </div>
                    </div>
                </section>
            </div>
        </div>
    </div>

    <!-- Automation Modal -->
    <div id="automation-modal" class="cds-modal-overlay">
        <div class="cds-modal">
            <div class="cds-modal-header">
                <span class="cds-modal-title">New Automation Rule</span>
                <button class="cds-modal-close" onclick="closeAutomationModal()">&times;</button>
            </div>
            <div class="cds-modal-body">
                <form id="automation-form">
                    <div class="cds-form-group">
                        <label class="cds-label">Rule Name *</label>
                        <input type="text" id="auto-name" class="cds-input" required placeholder="e.g. Stop at obstacle">
                    </div>
                    <div class="cds-grid cds-grid--2">
                        <div class="cds-form-group">
                            <label class="cds-label">Trigger Type</label>
                            <select id="auto-trigger" class="cds-input cds-select">
                                <option value="sensor">Sensor Threshold</option>
                                <option value="time">Time/Schedule</option>
                                <option value="manual">Manual Only</option>
                                <option value="webhook">Webhook</option>
                            </select>
                        </div>
                        <div class="cds-form-group">
                            <label class="cds-label">Condition Match</label>
                            <select id="auto-match" class="cds-input cds-select">
                                <option value="ALL">ALL (AND)</option>
                                <option value="ANY">ANY (OR)</option>
                            </select>
                        </div>
                    </div>
                    <div class="cds-form-group">
                        <label class="cds-label">Schedule (cron, optional)</label>
                        <input type="text" id="auto-schedule" class="cds-input" placeholder="*/30 * * * *">
                    </div>

                    <div class="cds-form-group">
                        <div class="flex justify-between items-center mb-4">
                            <label class="cds-label" style="margin:0;font-size:var(--cds-font-size-14);color:var(--cds-text-primary);font-weight:600">IF Conditions</label>
                            <button type="button" class="cds-btn cds-btn--ghost cds-btn--sm" onclick="addCondition(false)">+ Add</button>
                        </div>
                        <div id="if-conditions"></div>
                    </div>

                    <div class="cds-form-group">
                        <div class="flex justify-between items-center mb-4">
                            <label class="cds-label" style="margin:0;font-size:var(--cds-font-size-14);color:var(--cds-text-primary);font-weight:600">THEN Actions</label>
                            <button type="button" class="cds-btn cds-btn--ghost cds-btn--sm" onclick="addAction(false)">+ Add</button>
                        </div>
                        <div id="if-actions"></div>
                    </div>

                    <div class="cds-form-group">
                        <div class="flex justify-between items-center mb-4">
                            <label class="cds-label" style="margin:0;font-size:var(--cds-font-size-14);color:var(--cds-text-primary);font-weight:600">ELSE Conditions (optional)</label>
                            <button type="button" class="cds-btn cds-btn--ghost cds-btn--sm" onclick="addCondition(true)">+ Add</button>
                        </div>
                        <div id="else-conditions"></div>
                    </div>

                    <div class="cds-form-group">
                        <div class="flex justify-between items-center mb-4">
                            <label class="cds-label" style="margin:0;font-size:var(--cds-font-size-14);color:var(--cds-text-primary);font-weight:600">ELSE Actions (optional)</label>
                            <button type="button" class="cds-btn cds-btn--ghost cds-btn--sm" onclick="addAction(true)">+ Add</button>
                        </div>
                        <div id="else-actions"></div>
                    </div>
                </form>
            </div>
            <div class="cds-modal-footer">
                <button class="cds-btn cds-btn--ghost" onclick="closeAutomationModal()">Cancel</button>
                <button class="cds-btn cds-btn--primary" onclick="document.getElementById('automation-form').requestSubmit()">Create Rule</button>
            </div>
        </div>
    </div>

    <script>
        // --- Navigation ---
        let currentPage = 'dashboard';

        function navigateTo(page) {
            document.querySelectorAll('.page-section').forEach(s => s.classList.remove('active'));
            document.querySelectorAll('.nav-item').forEach(n => n.classList.remove('active'));
            document.getElementById('page-' + page).classList.add('active');
            document.querySelector('[data-page="' + page + '"]').classList.add('active');
            const titles = {
                dashboard: 'Dashboard', sensors: 'Sensors', movement: 'Movement',
                manipulation: 'Manipulation', pathplanning: 'Path Planning',
                automation: 'Automation Rules', serial: 'Serial Command', serialmonitor: 'Serial Monitor'
            };
            document.getElementById('page-title').textContent = titles[page] || page;
            currentPage = page;

            if (page === 'pathplanning') loadMap();
            if (page === 'serialmonitor') setTimeout(startSerialMonitor, 100);
            if (page !== 'serialmonitor' && currentSerialMonitorInterval) stopSerialMonitor();
        }

        function clearLog() { document.getElementById('robot-log').innerHTML = ''; }

        // --- Clock ---
        setInterval(() => {
            const now = new Date();
            document.getElementById('header-time').textContent =
                now.toLocaleTimeString('en-US', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });
        }, 1000);

        // --- Display updaters ---
        function updateSpeedDisplay() {
            document.getElementById('speed-display').textContent = document.getElementById('speed-slider').value + '%';
        }
        function updateWheelSpeedDisplay() {
            const v = document.getElementById('wheel-speed').value;
            document.getElementById('wheel-speed-display').textContent = v;
        }
        function updateGripperTiltDisplay() {
            document.getElementById('gripper-tilt-display').textContent = document.getElementById('gripper-tilt').value + '\u00B0';
        }

        // --- API helper ---
        async function apiCall(endpoint, data = {}, options = {}) {
            try {
                const resp = await fetch(endpoint, {
                    method: options.method || 'GET',
                    headers: { 'Content-Type': 'application/json', ...options.headers },
                    body: data && Object.keys(data).length ? JSON.stringify(data) : undefined
                });
                return await resp.json();
            } catch (e) {
                console.error('API error:', e);
                return { success: false, error: e.message };
            }
        }

        // --- Robot control functions (preserve all existing logic) ---
        async function setSpeed() {
            const speed = parseInt(document.getElementById('speed-slider').value);
            await apiCall('/api/robot/speed', { speed }, { method: 'POST' });
            addLog('Speed set to ' + speed + '%');
        }

        async function toggleTurbo() {
            const r = await apiCall('/api/robot/turbo', {}, { method: 'POST' });
            addLog('Turbo: ' + (r.turbo ? 'ON' : 'OFF'));
        }

        async function moveRobot(dir) {
            await apiCall('/api/robot/move', { direction: dir }, { method: 'POST' });
            addLog('Move: ' + dir);
        }

        async function turnRobot(dir) {
            await apiCall('/api/robot/turn', { direction: dir }, { method: 'POST' });
            addLog('Turn: ' + dir);
        }

        async function stopRobot() {
            await apiCall('/api/robot/stop', {}, { method: 'POST' });
            addLog('STOP');
        }

        async function controlWheel() {
            const id = parseInt(document.getElementById('wheel-select').value);
            const speed = parseInt(document.getElementById('wheel-speed').value);
            await apiCall('/api/robot/wheels/' + id, { speed }, { method: 'POST' });
        }

        async function stopAllWheels() {
            await apiCall('/api/robot/wheels/stop', {}, { method: 'POST' });
        }

        async function executePreset(preset) {
            await apiCall('/api/robot/presets/' + preset, {}, { method: 'POST' });
            addLog('Preset: ' + preset);
        }

        async function controlGripper(cmd) {
            await apiCall('/api/robot/picker/gripper', { command: cmd }, { method: 'POST' });
            addLog('Gripper: ' + cmd);
        }

        async function setGripperTilt() {
            const angle = parseInt(document.getElementById('gripper-tilt').value);
            await apiCall('/api/robot/picker/gripper_tilt', { angle }, { method: 'POST' });
        }

        async function controlContainer(action) {
            const id = document.getElementById('container-id').value;
            await apiCall('/api/robot/container/' + id + '/' + action, {}, { method: 'POST' });
        }

        async function emergencyStop() {
            await apiCall('/api/robot/emergency-stop', {}, { method: 'POST' });
            addLog('EMERGENCY STOP');
        }

        async function homeServos() {
            await apiCall('/api/robot/servos/home', {}, { method: 'POST' });
        }

        async function testLimitSwitches() {
            const r = await apiCall('/api/robot/limits/scan');
            addLog('Limit switches: ' + JSON.stringify(r));
        }

        // --- Serial ---
        async function sendSerialCommand() {
            const cmd = document.getElementById('serial-command').value.trim();
            if (!cmd) return;
            await apiCall('/api/serial/send', { command: cmd }, { method: 'POST' });
            addSerialLog('> ' + cmd);
            document.getElementById('serial-command').value = '';
        }

        async function sendQuickCommand(cmd) {
            await apiCall('/api/serial/send', { command: cmd }, { method: 'POST' });
            addSerialLog('> ' + cmd);
        }

        function addSerialLog(text) {
            const el = document.getElementById('serial-log');
            el.innerHTML += text + '\n';
            el.scrollTop = el.scrollHeight;
        }

        // --- Serial Monitor ---
        let currentSerialMonitorInterval = null;

        async function startSerialMonitor() {
            document.getElementById('monitor-start-btn').classList.add('hidden');
            document.getElementById('monitor-stop-btn').classList.remove('hidden');
            document.getElementById('monitor-connection-status').className = 'status-dot connected';
            document.getElementById('monitor-connection-text').textContent = 'Monitoring';
            currentSerialMonitorInterval = setInterval(fetchSerialData, 200);
        }

        async function stopSerialMonitor() {
            if (currentSerialMonitorInterval) clearInterval(currentSerialMonitorInterval);
            currentSerialMonitorInterval = null;
            document.getElementById('monitor-start-btn').classList.remove('hidden');
            document.getElementById('monitor-stop-btn').classList.add('hidden');
            document.getElementById('monitor-connection-status').className = 'status-dot';
            document.getElementById('monitor-connection-text').textContent = 'Stopped';
        }

        async function fetchSerialData() {
            try {
                const r = await fetch('/api/serial/monitor');
                const d = await r.json();
                if (d.data) addToSerialMonitor(d.data, 'data');
            } catch (e) {}
        }

        function addToSerialMonitor(text, type) {
            const el = document.getElementById('serial-monitor-output');
            const placeholder = el.querySelector('div');
            if (placeholder && placeholder.style.textAlign === 'center') el.innerHTML = '';
            const line = document.createElement('div');
            line.textContent = text;
            el.appendChild(line);
            if (document.getElementById('monitor-autoscroll').checked) el.scrollTop = el.scrollHeight;
        }

        function clearSerialMonitor() {
            document.getElementById('serial-monitor-output').innerHTML =
                '<div style="text-align:center;padding:var(--cds-spacing-08);color:var(--cds-text-placeholder)">Cleared</div>';
        }

        // --- Sensors ---
        async function refreshSensors() {
            try {
                const r = await fetch('/api/robot/sensors');
                const d = await r.json();
                if (d.success) updateSensorDisplay(d.data || d.sensor_data || d);
            } catch (e) { console.error('Sensor error:', e); }
        }

        function updateSensorDisplay(data) {
            if (!data) return;
            const ls = data.laser_sensors || {};
            const us = data.ultrasonic_sensors || {};
            const ls2 = data.line_sensors || {};
            const imu = data.imu || {};
            const tf = data.tf_luna || {};

            const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };
            set('s-lf', ls.left_front != null ? ls.left_front.toFixed(1) : '--');
            set('s-lb', ls.left_back != null ? ls.left_back.toFixed(1) : '--');
            set('s-rf', ls.right_front != null ? ls.right_front.toFixed(1) : '--');
            set('s-rb', ls.right_back != null ? ls.right_back.toFixed(1) : '--');
            set('s-bl', ls.back_left != null ? ls.back_left.toFixed(1) : '--');
            set('s-br', ls.back_right != null ? ls.back_right.toFixed(1) : '--');
            set('s-ultra-fl', us.front_left != null ? us.front_left.toFixed(1) : '--');
            set('s-ultra-fr', us.front_right != null ? us.front_right.toFixed(1) : '--');
            set('s-line-l', ls2.left != null ? (ls2.left ? 'ON' : 'OFF') : '--');
            set('s-line-c', ls2.center != null ? (ls2.center ? 'ON' : 'OFF') : '--');
            set('s-line-r', ls2.right != null ? (ls2.right ? 'ON' : 'OFF') : '--');

            if (imu.orientation) {
                const h = imu.orientation.z || 0;
                const hd = h > 180 ? h - 360 : h;
                set('s-imu-h', hd.toFixed(1));
                set('kpi-heading', hd.toFixed(1) + '\u00B0');
            }

            set('kpi-lf', ls.left_front != null ? ls.left_front.toFixed(0) + 'cm' : '--');
            set('kpi-rf', ls.right_front != null ? ls.right_front.toFixed(0) + 'cm' : '--');
        }

        setInterval(refreshSensors, 2000);

        // --- Status ---
        async function updateStatus() {
            try {
                const r = await fetch('/api/status');
                const d = await r.json();
                const s = d.data || d;
                document.getElementById('kpi-status').textContent = s.simulation_mode ? 'SIM' : 'LIVE';
                document.getElementById('kpi-status').className = 'kpi-value ' + (s.simulation_mode ? 'kpi-value--warn' : 'kpi-value--ok');

                const megaEl = document.getElementById('kpi-mega');
                megaEl.textContent = s.mega_connected ? 'Connected' : 'Disconnected';
                megaEl.className = 'kpi-value ' + (s.mega_connected ? 'kpi-value--ok' : 'kpi-value--error');

                const megaDot = document.getElementById('mega-dot');
                megaDot.className = 'status-dot ' + (s.mega_connected ? 'connected' : 'error');
                document.getElementById('mega-label').textContent = 'Mega: ' + (s.mega_connected ? 'OK' : 'OFF');

                document.getElementById('ros2-dot').className = 'status-dot ' + (s.ros2_services_available ? 'connected' : '');
            } catch (e) {}
        }

        setInterval(updateStatus, 5000);
        updateStatus();

        // --- Position ---
        async function updateDashboardPosition() {
            try {
                const r = await fetch('/api/robot/position');
                const d = await r.json();
                if (d.data) {
                    const p = d.data;
                    document.getElementById('kpi-position').textContent =
                        (p.position ? p.position[0].toFixed(1) + ', ' + p.position[1].toFixed(1) : '--');
                }
            } catch (e) {}
        }

        setInterval(updateDashboardPosition, 3000);

        // --- Log ---
        function addLog(msg) {
            const el = document.getElementById('robot-log');
            const t = new Date().toLocaleTimeString('en-US', { hour12: false });
            el.innerHTML += '<div>' + t + ' ' + msg + '</div>';
            el.scrollTop = el.scrollHeight;
        }

        // --- Waypoints ---
        let waypoints = [];

        function addWaypoint() {
            const x = parseFloat(document.getElementById('waypoint-x').value);
            const y = parseFloat(document.getElementById('waypoint-y').value);
            const z = parseFloat(document.getElementById('waypoint-z').value) || 0;
            if (isNaN(x) || isNaN(y)) return;
            waypoints.push({ x, y, z });
            updateWaypointDisplay();
        }

        function updateWaypointDisplay() {
            const el = document.getElementById('waypoint-list');
            el.innerHTML = waypoints.length ? waypoints.map((w, i) =>
                '<div style="display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid var(--cds-border-subtle)">' +
                '<span class="mono">#' + (i + 1) + ' (' + w.x + ', ' + w.y + ', ' + w.z + ')</span>' +
                '<button class="cds-btn cds-btn--ghost cds-btn--sm" onclick="waypoints.splice(' + i + ',1);updateWaypointDisplay()">X</button></div>'
            ).join('') : 'No waypoints';
        }

        function clearWaypoints() { waypoints = []; updateWaypointDisplay(); }

        async function navigateWaypoints() {
            if (!waypoints.length) return;
            await apiCall('/api/robot/waypoints/navigate', { waypoints }, { method: 'POST' });
            addLog('Navigating ' + waypoints.length + ' waypoints');
        }

        function previewWaypoints() {
            addLog('Preview: ' + waypoints.length + ' waypoints');
        }

        function optimizeWaypoints() {
            if (waypoints.length < 3) return;
            const cx = waypoints.reduce((s, w) => s + w.x, 0) / waypoints.length;
            const cy = waypoints.reduce((s, w) => s + w.y, 0) / waypoints.length;
            waypoints.sort((a, b) => Math.atan2(a.y - cy, a.x - cx) - Math.atan2(b.y - cy, b.x - cx));
            updateWaypointDisplay();
            addLog('Optimized route');
        }

        // --- Map ---
        function loadMap() { document.getElementById('map-container').innerHTML = '<canvas id="map-canvas" width="400" height="300" style="background:#161616"></canvas>'; }
        function clearMapCanvas() { loadMap(); }

        // --- Movement Sequences ---
        async function executeMovementSet(name) {
            await apiCall('/api/robot/sequences/execute', { name }, { method: 'POST' });
            addLog('Sequence: ' + name);
        }

        async function stopSequence() {
            await apiCall('/api/robot/sequences/stop', {}, { method: 'POST' });
            addLog('Sequence stopped');
        }

        // --- Mega ---
        async function reconnectMega() {
            const r = await apiCall('/api/mega/reconnect', {}, { method: 'POST' });
            addLog('Mega reconnect: ' + JSON.stringify(r));
        }

        // --- Automation ---
        async function refreshAutomations() {
            try {
                const r = await fetch('/api/automations');
                const d = await r.json();
                displayAutomations(d.automations || []);
            } catch (e) {}
        }

        function displayAutomations(autos) {
            const el = document.getElementById('automation-list');
            if (!autos.length) {
                el.innerHTML = '<div style="text-align:center;padding:var(--cds-spacing-08);color:var(--cds-text-placeholder)"><i class="ph-bold ph-lightning" style="font-size:48px;display:block;margin-bottom:var(--cds-spacing-03)"></i>No rules yet</div>';
                return;
            }
            el.innerHTML = autos.map(a => {
                const conds = (a.conditions || []).map(c => '<span class="cds-tag">' + c.feedKey + ' ' + c.operator + ' ' + c.value + '</span>').join(' ');
                const acts = (a.actions || []).map(ac => '<span class="cds-tag cds-tag--info">' + ac.actionType + (ac.actionValue ? ':' + ac.actionValue : '') + '</span>').join(' ');
                return '<div style="background:var(--cds-bg-tertiary);border:1px solid var(--cds-border-subtle);padding:var(--cds-spacing-04);margin-bottom:var(--cds-spacing-03);display:flex;justify-content:space-between;align-items:center">' +
                    '<div style="flex:1"><div style="font-weight:600;margin-bottom:4px">' + a.name + '</div><div style="display:flex;flex-wrap:wrap;gap:4px">' + conds + ' <span style="color:var(--cds-text-placeholder)">&rarr;</span> ' + acts + '</div></div>' +
                    '<div class="flex gap-2 items-center">' +
                    '<label class="cds-toggle"><input type="checkbox" ' + (a.isActive ? 'checked' : '') + ' onchange="toggleAutomation(' + a.id + ')"><span class="cds-toggle-slider"></span></label>' +
                    '<button class="cds-btn cds-btn--primary cds-btn--sm" onclick="runAutomation(' + a.id + ')"><i class="ph-bold ph-play"></i></button>' +
                    '<button class="cds-btn cds-btn--danger cds-btn--sm" onclick="deleteAutomation(' + a.id + ')"><i class="ph-bold ph-trash"></i></button>' +
                    '</div></div>';
            }).join('');
        }

        let condCount = 0, actCount = 0;

        function createAutomation() {
            document.getElementById('automation-modal').classList.add('open');
            document.getElementById('if-conditions').innerHTML = '';
            document.getElementById('if-actions').innerHTML = '';
            document.getElementById('else-conditions').innerHTML = '';
            document.getElementById('else-actions').innerHTML = '';
            condCount = 0; actCount = 0;
            addCondition(false);
            addAction(false);
        }

        function closeAutomationModal() { document.getElementById('automation-modal').classList.remove('open'); }

        function addCondition(isElse) {
            const container = isElse ? document.getElementById('else-conditions') : document.getElementById('if-conditions');
            const id = ++condCount;
            const feeds = ['laser_left_front','laser_left_back','laser_right_front','laser_right_back','laser_back_left','laser_back_right','ultra_front_left','ultra_front_right','line_left','line_center','line_right','imu_heading','imu_pitch','imu_roll','tf_luna_distance','mega_connected'];
            container.insertAdjacentHTML('beforeend',
                '<div id="cond-' + id + '" class="flex gap-2 items-center mb-4">' +
                '<select class="cds-input cds-select cond-feed" style="flex:2">' + feeds.map(f => '<option value="' + f + '">' + f + '</option>').join('') + '</select>' +
                '<select class="cds-input cds-select cond-op" style="flex:1"><option>></option><option><</option><option>>=</option><option><=</option><option>==</option><option>!=</option></select>' +
                '<input type="text" class="cds-input cond-val" placeholder="Value" style="flex:1">' +
                '<button type="button" class="cds-btn cds-btn--ghost cds-btn--sm" onclick="document.getElementById(\'cond-' + id + '\').remove()">X</button></div>'
            );
        }

        function addAction(isElse) {
            const container = isElse ? document.getElementById('else-actions') : document.getElementById('if-actions');
            const id = ++actCount;
            container.insertAdjacentHTML('beforeend',
                '<div id="act-' + id + '" class="flex gap-2 items-center mb-4">' +
                '<select class="cds-input cds-select act-type" style="flex:1" onchange="this.nextElementSibling.placeholder=({move:\'dir\',turn:\'dir\',gripper:\'open/close\',stop:\'\',speed:\'0-100\',delay:\'ms\',webhook:\'url\',trigger:\'id\'})[this.value]||\'Value\'">' +
                '<option>move</option><option>turn</option><option>gripper</option><option>stop</option><option>speed</option><option>delay</option><option>webhook</option><option>trigger</option></select>' +
                '<input type="text" class="cds-input act-val" placeholder="Value" style="flex:1">' +
                '<button type="button" class="cds-btn cds-btn--ghost cds-btn--sm" onclick="document.getElementById(\'act-' + id + '\').remove()">X</button></div>'
            );
        }

        document.getElementById('automation-form').addEventListener('submit', async function(e) {
            e.preventDefault();
            const conds = [];
            document.querySelectorAll('#if-conditions [id^=cond-]').forEach(d => {
                conds.push({ feedKey: d.querySelector('.cond-feed').value, operator: d.querySelector('.cond-op').value, value: d.querySelector('.cond-val').value, isElse: false });
            });
            document.querySelectorAll('#else-conditions [id^=cond-]').forEach(d => {
                conds.push({ feedKey: d.querySelector('.cond-feed').value, operator: d.querySelector('.cond-op').value, value: d.querySelector('.cond-val').value, isElse: true });
            });
            const acts = [];
            document.querySelectorAll('#if-actions [id^=act-]').forEach((d, i) => {
                acts.push({ actionType: d.querySelector('.act-type').value, actionValue: d.querySelector('.act-val').value, actionOrder: i, isElse: false });
            });
            document.querySelectorAll('#else-actions [id^=act-]').forEach((d, i) => {
                acts.push({ actionType: d.querySelector('.act-type').value, actionValue: d.querySelector('.act-val').value, actionOrder: i, isElse: true });
            });
            const auto = {
                name: document.getElementById('auto-name').value,
                triggerType: document.getElementById('auto-trigger').value,
                conditionMatch: document.getElementById('auto-match').value,
                scheduleCron: document.getElementById('auto-schedule').value || null,
                conditions: conds, actions: acts
            };
            const r = await fetch('/api/automations', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(auto) });
            if (r.ok) { closeAutomationModal(); refreshAutomations(); }
            else { const err = await r.json(); alert(err.error || 'Error'); }
        });

        async function toggleAutomation(id) {
            await fetch('/api/automations/' + id + '/toggle', { method: 'POST' });
            refreshAutomations();
        }

        async function runAutomation(id) {
            const r = await fetch('/api/automations/' + id + '/run', { method: 'POST' });
            const d = await r.json();
            addLog(d.triggered ? 'Automation triggered' : 'Not triggered');
        }

        async function deleteAutomation(id) {
            if (!confirm('Delete this rule?')) return;
            await fetch('/api/automations/' + id, { method: 'DELETE' });
            refreshAutomations();
        }

        // --- Init ---
        refreshAutomations();
    </script>
</body>
</html>
"""
