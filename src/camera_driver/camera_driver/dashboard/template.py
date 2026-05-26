"""
Operator console HTML template.
Edit the UI here without touching any logic.
"""

DASHBOARD_HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>Robot Operator Console</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600;700&family=DM+Sans:wght@400;500;600&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-base: #111113;
      --bg-surface: #18181b;
      --bg-raised: #1e1e22;
      --bg-hover: #26262b;
      --border: #2a2a30;
      --border-light: #333339;
      --text-primary: #e4e4e7;
      --text-secondary: #8b8b95;
      --text-muted: #5c5c66;
      --accent: #3b82f6;
      --accent-dim: #2563eb;
      --accent-glow: rgba(59, 130, 246, 0.15);
      --warning: #f59e0b;
      --danger: #ef4444;
      --success: #22c55e;
      --front-cam: #f59e0b;
      --rear-cam: #3b82f6;
      --robot-ops: #22c55e;
    }

    * { box-sizing: border-box; margin: 0; padding: 0; }

    body {
      font-family: 'DM Sans', sans-serif;
      background: var(--bg-base);
      color: var(--text-primary);
      min-height: 100vh;
      overflow: hidden;
    }

    /* ─── HEADER ─── */
    .header {
      height: 48px;
      background: var(--bg-surface);
      border-bottom: 1px solid var(--border);
      display: flex;
      align-items: center;
      padding: 0 20px;
      gap: 16px;
    }
    .header-brand {
      font-family: 'JetBrains Mono', monospace;
      font-size: 13px;
      font-weight: 600;
      color: var(--text-primary);
      letter-spacing: 0.5px;
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .header-brand .dot {
      width: 8px; height: 8px;
      background: var(--success);
      border-radius: 50%;
      box-shadow: 0 0 6px var(--success);
    }
    .header-spacer { flex: 1; }
    .header-status {
      font-family: 'JetBrains Mono', monospace;
      font-size: 11px;
      color: var(--text-muted);
      display: flex;
      gap: 16px;
      align-items: center;
    }
    .status-item {
      display: flex;
      align-items: center;
      gap: 5px;
    }
    .status-dot {
      width: 6px; height: 6px;
      border-radius: 50%;
    }
    .status-dot.live { background: var(--success); box-shadow: 0 0 4px var(--success); }
    .status-dot.offline { background: var(--text-muted); }
    .rec-badge {
      display: none;
      background: var(--danger);
      color: #fff;
      font-size: 10px;
      padding: 2px 8px;
      border-radius: 3px;
      font-family: 'JetBrains Mono', monospace;
      animation: rec-pulse 1.5s infinite;
    }
    .rec-badge.active { display: inline-block; }
    @keyframes rec-pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }

    /* ─── LAYOUT ─── */
    .layout {
      display: flex;
      height: calc(100vh - 48px);
    }

    /* ─── STREAMS COLUMN ─── */
    .streams {
      flex: 1;
      display: flex;
      flex-direction: column;
      background: #0a0a0c;
      min-width: 0;
    }
    .stream-wrapper {
      flex: 1;
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
      min-height: 0;
    }
    .stream-wrapper + .stream-wrapper {
      border-top: 1px solid var(--border);
    }
    .stream-wrapper img {
      max-width: 100%;
      max-height: 100%;
      object-fit: contain;
    }
    .stream-tag {
      position: absolute;
      top: 12px;
      left: 12px;
      font-family: 'JetBrains Mono', monospace;
      font-size: 10px;
      font-weight: 600;
      letter-spacing: 1.5px;
      text-transform: uppercase;
      padding: 4px 10px;
      border-radius: 3px;
      background: rgba(0,0,0,0.6);
      backdrop-filter: blur(4px);
    }
    .stream-tag.front { color: var(--front-cam); border: 1px solid rgba(245,158,11,0.3); }
    .stream-tag.rear { color: var(--rear-cam); border: 1px solid rgba(59,130,246,0.3); }

    .stream-offline {
      text-align: center;
      font-family: 'JetBrains Mono', monospace;
      font-size: 12px;
      color: var(--text-muted);
      line-height: 1.8;
    }
    .stream-offline .hint {
      font-size: 10px;
      color: var(--border-light);
    }

    /* ─── PANEL ─── */
    .panel {
      width: 380px;
      min-width: 380px;
      background: var(--bg-surface);
      border-left: 1px solid var(--border);
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }

    /* ─── TABS ─── */
    .tabs {
      display: flex;
      border-bottom: 1px solid var(--border);
      background: var(--bg-surface);
      flex-shrink: 0;
    }
    .tab {
      flex: 1;
      padding: 12px 0;
      text-align: center;
      font-family: 'JetBrains Mono', monospace;
      font-size: 10px;
      font-weight: 500;
      letter-spacing: 1px;
      text-transform: uppercase;
      color: var(--text-muted);
      cursor: pointer;
      border-bottom: 2px solid transparent;
      transition: all 0.15s;
      user-select: none;
    }
    .tab:hover { color: var(--text-secondary); background: var(--bg-raised); }
    .tab.active-front { color: var(--front-cam); border-bottom-color: var(--front-cam); }
    .tab.active-rear { color: var(--rear-cam); border-bottom-color: var(--rear-cam); }
    .tab.active-robot { color: var(--robot-ops); border-bottom-color: var(--robot-ops); }

    /* ─── TAB CONTENT ─── */
    .tab-content {
      flex: 1;
      overflow-y: auto;
      padding: 16px;
    }
    .tab-pane { display: none; }
    .tab-pane.active { display: block; }

    /* ─── CONTROLS ─── */
    .section {
      margin-bottom: 20px;
    }
    .section-title {
      font-family: 'JetBrains Mono', monospace;
      font-size: 10px;
      font-weight: 600;
      letter-spacing: 1.5px;
      text-transform: uppercase;
      color: var(--text-muted);
      margin-bottom: 12px;
      padding-bottom: 6px;
      border-bottom: 1px solid var(--border);
    }
    .section-title.front { color: var(--front-cam); border-color: rgba(245,158,11,0.2); }
    .section-title.rear { color: var(--rear-cam); border-color: rgba(59,130,246,0.2); }
    .section-title.robot { color: var(--robot-ops); border-color: rgba(34,197,94,0.2); }

    .control-row {
      display: flex;
      align-items: center;
      margin-bottom: 10px;
    }
    .control-row label {
      width: 120px;
      font-size: 12px;
      color: var(--text-secondary);
      flex-shrink: 0;
    }
    .control-row input[type=range] {
      flex: 1;
      height: 4px;
      -webkit-appearance: none;
      appearance: none;
      background: var(--border);
      border-radius: 2px;
      outline: none;
    }
    .control-row input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 14px; height: 14px;
      border-radius: 50%;
      background: var(--accent);
      cursor: pointer;
      border: 2px solid var(--bg-surface);
    }
    .control-row .val {
      width: 50px;
      text-align: right;
      font-family: 'JetBrains Mono', monospace;
      font-size: 11px;
      color: var(--accent);
      flex-shrink: 0;
      margin-left: 8px;
    }

    select {
      background: var(--bg-raised);
      color: var(--text-primary);
      border: 1px solid var(--border);
      padding: 6px 10px;
      border-radius: 4px;
      font-family: 'DM Sans', sans-serif;
      font-size: 12px;
      cursor: pointer;
      outline: none;
    }
    select:focus { border-color: var(--accent); }

    /* ─── TOGGLE ─── */
    .toggle-row {
      display: flex;
      align-items: center;
      margin-bottom: 10px;
    }
    .toggle-row label {
      width: 120px;
      font-size: 12px;
      color: var(--text-secondary);
    }
    .toggle {
      position: relative;
      width: 36px;
      height: 18px;
      display: inline-block;
    }
    .toggle input {
      opacity: 0;
      width: 100%;
      height: 100%;
      position: absolute;
      top: 0; left: 0;
      z-index: 2;
      cursor: pointer;
      margin: 0;
    }
    .toggle .sw {
      position: absolute;
      top: 0; left: 0; right: 0; bottom: 0;
      background: var(--border);
      border-radius: 9px;
      transition: 0.2s;
      z-index: 1;
    }
    .toggle .sw:before {
      content: "";
      position: absolute;
      width: 12px; height: 12px;
      left: 3px; bottom: 3px;
      background: var(--text-muted);
      border-radius: 50%;
      transition: 0.2s;
    }
    .toggle input:checked + .sw {
      background: var(--accent);
    }
    .toggle input:checked + .sw:before {
      transform: translateX(18px);
      background: #fff;
    }

    /* ─── BUTTONS ─── */
    .btn-row {
      display: flex;
      gap: 8px;
      margin-top: 8px;
    }
    button {
      flex: 1;
      background: var(--bg-raised);
      color: var(--text-primary);
      border: 1px solid var(--border);
      padding: 8px 12px;
      border-radius: 4px;
      font-family: 'DM Sans', sans-serif;
      font-size: 12px;
      cursor: pointer;
      transition: all 0.15s;
    }
    button:hover { background: var(--bg-hover); border-color: var(--border-light); }
    button.rec-active {
      background: var(--danger);
      border-color: var(--danger);
      color: #fff;
    }

    /* ─── SCROLLBAR ─── */
    .tab-content::-webkit-scrollbar { width: 4px; }
    .tab-content::-webkit-scrollbar-track { background: transparent; }
    .tab-content::-webkit-scrollbar-thumb { background: var(--border); border-radius: 2px; }

    /* ─── RESPONSIVE ─── */
    @media (max-width: 900px) {
      .layout { flex-direction: column; }
      .streams { min-height: 300px; }
      .panel { width: 100%; min-width: 0; border-left: none; border-top: 1px solid var(--border); }
    }
  </style>
</head>
<body>

  <!-- ═══ HEADER ═══ -->
  <div class="header">
    <div class="header-brand">
      <span class="dot" id="main-dot"></span>
      OPERATOR CONSOLE
    </div>
    <div class="header-spacer"></div>
    <span class="rec-badge" id="rec-indicator">● REC</span>
    <div class="header-status">
      <span class="status-item">
        <span class="status-dot live" id="front-dot"></span>
        FRONT
      </span>
      <span class="status-item">
        <span class="status-dot offline" id="rear-dot"></span>
        REAR
      </span>
    </div>
  </div>

  <!-- ═══ LAYOUT ═══ -->
  <div class="layout">

    <!-- ─── STREAMS ─── -->
    <div class="streams">
      <div class="stream-wrapper">
        <span class="stream-tag front">FRONT</span>
        <img id="front-stream" src="/video/both" alt="Front Camera" />
      </div>
      <div class="stream-wrapper">
        <span class="stream-tag rear">REAR</span>
        <img id="rear-stream" alt="Rear Camera"
             onerror="this.style.display='none'; document.getElementById('rear-offline').style.display='block'"
             onload="this.style.display='block'; document.getElementById('rear-offline').style.display='none'" />
        <div id="rear-offline" class="stream-offline">
          REAR CAMERA OFFLINE<br>
          <span class="hint">Waiting for rear camera service...</span>
        </div>
      </div>
    </div>

    <!-- ─── PANEL ─── -->
    <div class="panel">

      <!-- TABS -->
      <div class="tabs">
        <div class="tab active-front" data-tab="front" onclick="switchTab('front')">Front Cam</div>
        <div class="tab" data-tab="rear" onclick="switchTab('rear')">Rear Cam</div>
        <div class="tab" data-tab="robot" onclick="switchTab('robot')">Robot</div>
      </div>

      <!-- TAB CONTENT -->
      <div class="tab-content">

        <!-- ════ FRONT CAMERA TAB ════ -->
        <div class="tab-pane active" id="pane-front">

          <div class="section">
            <div class="section-title front">Stream</div>
            <div class="control-row">
              <label>View</label>
              <select id="f-stream-mode" onchange="switchFrontStream(this.value)">
                <option value="both" selected>RGB + Depth</option>
                <option value="rgb">RGB Only</option>
                <option value="depth">Depth Only</option>
              </select>
            </div>
            <div class="control-row">
              <label>Resolution</label>
              <select id="f-resolution" onchange="frontRestart('resolution', this.value)">
                <option value="480p">480p (cropped)</option>
                <option value="720p">720p</option>
                <option value="1080p" selected>1080p (native)</option>
              </select>
            </div>
            <div class="control-row">
              <label>FPS</label>
              <select id="f-fps" onchange="frontRestart('fps', parseInt(this.value))">
                <option value="10">10</option>
                <option value="15">15</option>
                <option value="24">24</option>
                <option value="30" selected>30</option>
                <option value="60">60</option>
              </select>
            </div>
            <div class="control-row">
              <label>JPEG Quality</label>
              <input type="range" min="30" max="100" value="80"
                     oninput="frontCtrl('jpeg_quality', parseInt(this.value), this)">
              <span class="val" id="fv_jpeg_quality">80</span>
            </div>
          </div>

          <div class="section">
            <div class="section-title front">Recording</div>
            <div class="btn-row">
              <button id="f-rec-btn" onclick="toggleFrontRec()">● Start Recording</button>
            </div>
          </div>

          <div class="section">
            <div class="section-title front">Exposure</div>
            <div class="toggle-row">
              <label>Auto Exposure</label>
              <div class="toggle">
                <input type="checkbox" checked onchange="frontCtrl('auto_exposure', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
            <div class="control-row">
              <label>Exposure (µs)</label>
              <input type="range" min="1" max="33000" value="8333" step="100"
                     oninput="frontCtrl('exposure_us', parseInt(this.value), this)">
              <span class="val" id="fv_exposure_us">8333</span>
            </div>
            <div class="control-row">
              <label>ISO</label>
              <input type="range" min="100" max="1600" value="400" step="50"
                     oninput="frontCtrl('iso', parseInt(this.value), this)">
              <span class="val" id="fv_iso">400</span>
            </div>
          </div>

          <div class="section">
            <div class="section-title front">White Balance</div>
            <div class="toggle-row">
              <label>Auto WB</label>
              <div class="toggle">
                <input type="checkbox" checked onchange="frontCtrl('auto_white_balance', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
            <div class="control-row">
              <label>Color Temp (K)</label>
              <input type="range" min="1000" max="12000" value="5500" step="100"
                     oninput="frontCtrl('white_balance_k', parseInt(this.value), this)">
              <span class="val" id="fv_white_balance_k">5500</span>
            </div>
          </div>

          <div class="section">
            <div class="section-title front">Image</div>
            <div class="control-row">
              <label>Brightness</label>
              <input type="range" min="-10" max="10" value="0"
                     oninput="frontCtrl('brightness', parseInt(this.value), this)">
              <span class="val" id="fv_brightness">0</span>
            </div>
            <div class="control-row">
              <label>Contrast</label>
              <input type="range" min="-10" max="10" value="0"
                     oninput="frontCtrl('contrast', parseInt(this.value), this)">
              <span class="val" id="fv_contrast">0</span>
            </div>
            <div class="control-row">
              <label>Saturation</label>
              <input type="range" min="-10" max="10" value="0"
                     oninput="frontCtrl('saturation', parseInt(this.value), this)">
              <span class="val" id="fv_saturation">0</span>
            </div>
            <div class="control-row">
              <label>Sharpness</label>
              <input type="range" min="0" max="4" value="1"
                     oninput="frontCtrl('sharpness', parseInt(this.value), this)">
              <span class="val" id="fv_sharpness">1</span>
            </div>
            <div class="control-row">
              <label>Luma Denoise</label>
              <input type="range" min="0" max="4" value="1"
                     oninput="frontCtrl('luma_denoise', parseInt(this.value), this)">
              <span class="val" id="fv_luma_denoise">1</span>
            </div>
            <div class="control-row">
              <label>Chroma Denoise</label>
              <input type="range" min="0" max="4" value="1"
                     oninput="frontCtrl('chroma_denoise', parseInt(this.value), this)">
              <span class="val" id="fv_chroma_denoise">1</span>
            </div>
          </div>

          <div class="section">
            <div class="section-title front">IR Illumination</div>
            <div class="control-row">
              <label>Dot Projector</label>
              <input type="range" min="0" max="1" value="0" step="0.01"
                     oninput="frontCtrl('ir_dot_brightness', parseFloat(this.value), this)">
              <span class="val" id="fv_ir_dot_brightness">0.00</span>
            </div>
            <div class="control-row">
              <label>Flood Light</label>
              <input type="range" min="0" max="1" value="0" step="0.01"
                     oninput="frontCtrl('ir_flood_brightness', parseFloat(this.value), this)">
              <span class="val" id="fv_ir_flood_brightness">0.00</span>
            </div>
          </div>

          <div class="section">
            <div class="section-title front">Stereo Depth</div>
            <div class="toggle-row">
              <label>Depth Enabled</label>
              <div class="toggle">
                <input type="checkbox" checked onchange="frontRestart('enable_depth', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
            <div class="control-row">
              <label>Confidence</label>
              <input type="range" min="0" max="255" value="200"
                     oninput="frontRestart('confidence_threshold', parseInt(this.value)); this.parentNode.querySelector('.val').textContent=this.value">
              <span class="val">200</span>
            </div>
            <div class="control-row">
              <label>Median Filter</label>
              <select onchange="frontRestart('median_filter', this.value)">
                <option value="OFF">Off</option>
                <option value="KERNEL_3x3">3×3</option>
                <option value="KERNEL_5x5">5×5</option>
                <option value="KERNEL_7x7" selected>7×7</option>
              </select>
            </div>
            <div class="toggle-row">
              <label>LR Check</label>
              <div class="toggle">
                <input type="checkbox" checked onchange="frontRestart('lr_check', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
            <div class="toggle-row">
              <label>Ext. Disparity</label>
              <div class="toggle">
                <input type="checkbox" onchange="frontRestart('extended_disparity', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
            <div class="toggle-row">
              <label>Subpixel</label>
              <div class="toggle">
                <input type="checkbox" onchange="frontRestart('subpixel', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
          </div>

          <div class="section">
            <div class="section-title front">Overlay</div>
            <div class="toggle-row">
              <label>Show FPS</label>
              <div class="toggle">
                <input type="checkbox" checked onchange="frontCtrl('show_fps', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
            <div class="toggle-row">
              <label>Timestamp</label>
              <div class="toggle">
                <input type="checkbox" onchange="frontCtrl('show_timestamp', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
          </div>

          <div class="section">
            <div class="btn-row">
              <button onclick="frontSnapshot()">Snapshot</button>
              <button onclick="frontReset()">Reset</button>
            </div>
          </div>

        </div>

        <!-- ════ REAR CAMERA TAB ════ -->
        <div class="tab-pane" id="pane-rear">

          <div class="section">
            <div class="section-title rear">Stream</div>
            <div class="control-row">
              <label>Resolution</label>
              <select id="r-resolution" onchange="rearRestart('resolution', this.value)">
                <option value="1640x1232 (4:3 full)">1640×1232 Binned</option>
                <option value="3280x2464 (4:3 max)" selected>3280×2464 Full</option>
                <option value="1920x1080 (16:9 crop)">1920×1080 16:9</option>
                <option value="640x480 (4:3 crop)">640×480 Crop</option>
                <option value="820x616 (4:3 scaled)">820×616 Scaled</option>
              </select>
            </div>
            <div class="control-row">
              <label>FPS</label>
              <select id="r-fps" onchange="rearRestart('fps', parseInt(this.value))">
                <option value="10">10</option>
                <option value="15">15</option>
                <option value="20" selected>20</option>
                <option value="30">30</option>
              </select>
            </div>
            <div class="control-row">
              <label>JPEG Quality</label>
              <input type="range" min="30" max="100" value="80"
                     oninput="rearCtrl('jpeg_quality', parseInt(this.value), this)">
              <span class="val" id="rv_jpeg_quality">80</span>
            </div>
          </div>

          <div class="section">
            <div class="section-title rear">Exposure</div>
            <div class="toggle-row">
              <label>Auto Exposure</label>
              <div class="toggle">
                <input type="checkbox" checked onchange="rearCtrl('auto_exposure', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
            <div class="control-row">
              <label>Exposure (µs)</label>
              <input type="range" min="100" max="120000" value="33000" step="100"
                     oninput="rearCtrl('exposure_time', parseInt(this.value), this)">
              <span class="val" id="rv_exposure_time">33000</span>
            </div>
            <div class="control-row">
              <label>Gain</label>
              <input type="range" min="1.0" max="16.0" value="1.0" step="0.5"
                     oninput="rearCtrl('analogue_gain', parseFloat(this.value), this)">
              <span class="val" id="rv_analogue_gain">1.0</span>
            </div>
          </div>

          <div class="section">
            <div class="section-title rear">White Balance</div>
            <div class="toggle-row">
              <label>Auto WB</label>
              <div class="toggle">
                <input type="checkbox" checked onchange="rearCtrl('auto_wb', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
            <div class="control-row">
              <label>Red Gain</label>
              <input type="range" min="0.5" max="4.0" value="1.5" step="0.1"
                     oninput="rearCtrl('wb_red_gain', parseFloat(this.value), this)">
              <span class="val" id="rv_wb_red_gain">1.5</span>
            </div>
            <div class="control-row">
              <label>Blue Gain</label>
              <input type="range" min="0.5" max="4.0" value="1.5" step="0.1"
                     oninput="rearCtrl('wb_blue_gain', parseFloat(this.value), this)">
              <span class="val" id="rv_wb_blue_gain">1.5</span>
            </div>
          </div>

          <div class="section">
            <div class="section-title rear">Image</div>
            <div class="control-row">
              <label>Brightness</label>
              <input type="range" min="-1.0" max="1.0" value="0.0" step="0.05"
                     oninput="rearCtrl('brightness', parseFloat(this.value), this)">
              <span class="val" id="rv_brightness">0.0</span>
            </div>
            <div class="control-row">
              <label>Contrast</label>
              <input type="range" min="0.0" max="2.0" value="1.0" step="0.05"
                     oninput="rearCtrl('contrast', parseFloat(this.value), this)">
              <span class="val" id="rv_contrast">1.0</span>
            </div>
            <div class="control-row">
              <label>Saturation</label>
              <input type="range" min="0.0" max="2.0" value="1.0" step="0.05"
                     oninput="rearCtrl('saturation', parseFloat(this.value), this)">
              <span class="val" id="rv_saturation">1.0</span>
            </div>
            <div class="control-row">
              <label>Sharpness</label>
              <input type="range" min="0.0" max="4.0" value="1.0" step="0.1"
                     oninput="rearCtrl('sharpness', parseFloat(this.value), this)">
              <span class="val" id="rv_sharpness">1.0</span>
            </div>
          </div>

          <div class="section">
            <div class="section-title rear">Orientation</div>
            <div class="toggle-row">
              <label>H-Flip</label>
              <div class="toggle">
                <input type="checkbox" onchange="rearRestart('hflip', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
            <div class="toggle-row">
              <label>V-Flip</label>
              <div class="toggle">
                <input type="checkbox" onchange="rearRestart('vflip', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
          </div>

          <div class="section">
            <div class="section-title rear">Overlay</div>
            <div class="toggle-row">
              <label>Show FPS</label>
              <div class="toggle">
                <input type="checkbox" checked onchange="rearCtrl('show_fps', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
            <div class="toggle-row">
              <label>Timestamp</label>
              <div class="toggle">
                <input type="checkbox" onchange="rearCtrl('show_timestamp', this.checked)">
                <span class="sw"></span>
              </div>
            </div>
          </div>

          <div class="section">
            <div class="btn-row">
              <button onclick="rearSnapshot()">Snapshot</button>
              <button onclick="rearReset()">Reset</button>
            </div>
          </div>

        </div>

        <!-- ════ ROBOT OPS TAB ════ -->
        <div class="tab-pane" id="pane-robot">
          <div class="section">
            <div class="section-title robot">Status</div>
            <div style="color: var(--text-muted); font-size: 13px; line-height: 2;">
              Battery, joystick mapping, LED controls, and other robot operations will go here.
            </div>
          </div>
        </div>

      </div>
    </div>
  </div>

  <!-- ═══ SCRIPT ═══ -->
  <script>
    // ── Config ──
    const FRONT_BASE = '';  // same origin (port 8080)
    const rearHost = window.location.hostname;
    const REAR_BASE = 'http://' + rearHost + ':8081';

    // ── Tabs ──
    const tabColors = { front: 'active-front', rear: 'active-rear', robot: 'active-robot' };
    function switchTab(name) {
      document.querySelectorAll('.tab').forEach(t => {
        t.className = 'tab' + (t.dataset.tab === name ? ' ' + tabColors[name] : '');
      });
      document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
      document.getElementById('pane-' + name).classList.add('active');
    }

    // ── Front camera controls ──
    let _frontTimer = null;
    function frontCtrl(key, value, el) {
      const s = document.getElementById('fv_' + key);
      if (s) s.textContent = typeof value === 'number' ?
        (Number.isInteger(value) ? value : value.toFixed(2)) : value;

      const send = () => fetch(FRONT_BASE + '/api/controls', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({[key]: value})
      });
      if (typeof value === 'boolean') { send(); }
      else { clearTimeout(_frontTimer); _frontTimer = setTimeout(send, 150); }
    }
    function frontRestart(key, value) {
      // If disabling depth, switch view to RGB only
      if (key === 'enable_depth' && value === false) {
        const sel = document.getElementById('f-stream-mode');
        if (sel) sel.value = 'rgb';
      }
      fetch(FRONT_BASE + '/api/restart', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({[key]: value})
      }).then(() => {
        setTimeout(() => {
          const mode = document.getElementById('f-stream-mode').value;
          document.getElementById('front-stream').src = '/video/' + mode + '?' + Date.now();
        }, 3000);
      });
    }
    function switchFrontStream(mode) {
      document.getElementById('front-stream').src = '/video/' + mode + '?' + Date.now();
    }
    function frontSnapshot() { window.open(FRONT_BASE + '/snapshot', '_blank'); }
    function frontReset() {
      fetch(FRONT_BASE + '/api/reset', {method: 'POST'}).then(() => location.reload());
    }
    function toggleFrontRec() {
      fetch(FRONT_BASE + '/api/recording/toggle', {method: 'POST'})
        .then(r => r.json())
        .then(data => {
          const btn = document.getElementById('f-rec-btn');
          const ind = document.getElementById('rec-indicator');
          if (data.recording) {
            btn.textContent = '■ Stop Recording';
            btn.classList.add('rec-active');
            ind.classList.add('active');
          } else {
            btn.textContent = '● Start Recording';
            btn.classList.remove('rec-active');
            ind.classList.remove('active');
          }
        });
    }

    // ── Rear camera controls ──
    let _rearTimer = null;
    function rearCtrl(key, value, el) {
      const s = document.getElementById('rv_' + key);
      if (s) s.textContent = typeof value === 'number' ?
        (Number.isInteger(value) ? value : value.toFixed(1)) : value;

      const send = () => fetch(REAR_BASE + '/api/controls', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({[key]: value})
      });
      if (typeof value === 'boolean') { send(); }
      else { clearTimeout(_rearTimer); _rearTimer = setTimeout(send, 150); }
    }
    function rearRestart(key, value) {
      fetch(REAR_BASE + '/api/restart', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({[key]: value})
      }).then(() => {
        setTimeout(() => {
          document.getElementById('rear-stream').src = REAR_BASE + '/video/rear?' + Date.now();
        }, 3000);
      });
    }
    function rearSnapshot() { window.open(REAR_BASE + '/snapshot', '_blank'); }
    function rearReset() {
      fetch(REAR_BASE + '/api/reset', {method: 'POST'}).then(() => location.reload());
    }

    // ── Status polling ──
    function pollFront() {
      fetch(FRONT_BASE + '/status').then(r => r.json()).then(() => {
        document.getElementById('front-dot').className = 'status-dot live';
      }).catch(() => {
        document.getElementById('front-dot').className = 'status-dot offline';
      });
    }
    function connectRear() {
      fetch(REAR_BASE + '/status', {mode: 'cors'}).then(r => r.json()).then(() => {
        const img = document.getElementById('rear-stream');
        if (!img.src || img.src === '') img.src = REAR_BASE + '/video/rear';
        document.getElementById('rear-dot').className = 'status-dot live';
      }).catch(() => {
        document.getElementById('rear-dot').className = 'status-dot offline';
      });
    }

    setInterval(pollFront, 3000);
    setInterval(connectRear, 5000);
    pollFront();
    connectRear();
  </script>
</body>
</html>

"""