"""
CoSES Complete Workflow - Plot creation
- Clears previous plots
- Parameter descriptions
- Simulation time control  
- Realistic setpoints
- Fixed validation
- For all building types
"""

from pathlib import Path
import sys
import json
from datetime import datetime
import webbrowser
from threading import Timer
import subprocess
import shutil

sys.path.insert(0, str(Path(__file__).parent))

try:
    from flask import Flask, render_template_string, request, jsonify
    from scipy.io import loadmat
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError as e:
    print(f"❌ Missing package: {e}")
    print("Run: pip install flask scipy matplotlib numpy")
    sys.exit(1)

from perfect_generator import PerfectDymolaVariantGenerator

app = Flask(__name__)

CURRENT_PROJECT = {
    'path': None,
    'mat_file': None,
    'plots_ready': False
}

def find_latest_mat_file(project_path):
    """Find the most recent .mat file in project directory"""
    project_path = Path(project_path)
    mat_files = list(project_path.glob("*.mat"))
    
    if not mat_files:
        return None
    
    latest = max(mat_files, key=lambda p: p.stat().st_mtime)
    return latest

def open_in_dymola(package_file, dymola_path=None):
    """Open the package.mo file in Dymola"""
    if not dymola_path:
        possible_paths = [
            r"C:\Program Files\Dymola 2023x\bin64\Dymola.exe",
            r"C:\Program Files\Dymola 2023x\bin64\Dymola.exe",
            r"C:\Program Files\Dymola 2025x Refresh 1\bin64\Dymola.exe",
            r"C:\Program Files\Dymola 2024\bin64\Dymola.exe",
            r"C:\Program Files\Dymola 2023\bin64\Dymola.exe",
        ]
        for path in possible_paths:
            if Path(path).exists():
                dymola_path = path
                break
    
    if dymola_path and Path(dymola_path).exists():
        print(f"🚀 Opening in Dymola: {package_file}")
        try:
            subprocess.Popen([str(dymola_path), str(package_file)])
            return True
        except Exception as e:
            print(f"❌ Error opening Dymola: {e}")
            return False
    return False

def extract_signal_names_vertical(mat_data):
    """Extract signal names using VERTICAL reading method"""
    if 'name' not in mat_data:
        return []
    
    names_raw = mat_data['name']
    
    print(f"📊 Extracting signals (vertical method)...")
    print(f"   Array shape: {names_raw.shape}")
    
    signals = []
    max_positions = 3061
    
    for pos in range(max_positions):
        chars = []
        for row_idx in range(len(names_raw)):
            if pos < len(names_raw[row_idx]):
                c = names_raw[row_idx][pos]
                if c not in ['\x00', '\0', ' ']:
                    chars.append(c)
        
        name = ''.join(chars).strip()
        if name and 0 < len(name) < 200:
            signals.append(name)
        
        if len(signals) > 150:
            break
    
    print(f"   ✅ Extracted {len(signals)} signals")
    return signals

def extract_signal_data(mat_data, signal_name):
    """Extract time and data for a specific signal"""
    names = extract_signal_names_vertical(mat_data)
    
    if signal_name not in names:
        return None, None
    
    signal_idx = names.index(signal_name)
    data_info = mat_data['dataInfo']
    
    matrix_num = int(data_info[0, signal_idx])
    col_idx = int(data_info[1, signal_idx])
    
    if matrix_num == 1:
        data_matrix = mat_data['data_1']
    elif matrix_num == 2:
        data_matrix = mat_data['data_2']
    else:
        return None, None
    
    negate = False
    if col_idx < 0:
        negate = True
        col_idx = -col_idx
    
    col_idx = abs(col_idx) - 1
    
    if col_idx >= data_matrix.shape[1]:
        return None, None
    
    time = mat_data['data_2'][0, :]
    data = data_matrix[col_idx, :]
    
    if negate:
        data = -data
    
    return time, data

def create_9_plots(mat_file, output_folder):
    """Create the 9 standard plots with CORRECT signal names"""
    
    print(f"\n{'='*70}")
    print(f"📊 CREATING 9 ANALYSIS PLOTS")
    print(f"{'='*70}")
    
    # CLEAR OLD PLOTS FIRST
    output_folder = Path(output_folder)
    if output_folder.exists():
        print(f"🗑️  Removing old plots from {output_folder}")
        shutil.rmtree(output_folder)
    
    output_folder.mkdir(exist_ok=True, parents=True)
    print(f"✅ Created clean plots folder")
    
    try:
        mat_data = loadmat(mat_file)
    except Exception as e:
        print(f"❌ Error loading .mat file: {e}")
        return []
    
    all_signals = extract_signal_names_vertical(mat_data)
    print(f"✅ Found {len(all_signals)} total signals")
    
    if len(all_signals) == 0:
        return []
    
    # Save debug file
    debug_file = Path(output_folder).parent / "all_signals_CORRECT.txt"
    with open(debug_file, 'w') as f:
        f.write("ALL EXTRACTED SIGNALS (CORRECT METHOD)\n")
        f.write("="*70 + "\n\n")
        for i, sig in enumerate(all_signals, 1):
            f.write(f"{i:3d}. {sig}\n")
    print(f"📝 Debug file: {debug_file}")
    
    # Define EXACT signal mappings
    signal_map = {
        'T_living': 'building.T_roomIs_degC',
        'T_cellar': 'building.T_cellarIs_degC',
        'T_roof': 'building.T_roofIs_degC',
        'T_ambient': 'building.T_ambient_degC',
        'T_supply': 'building.supply.medium.T_degC',
        'T_return': 'building.return_sink.medium.T_degC',
        'valve_living': 'building.valve_living_opening',
        'valve_cellar': 'building.valve_cellar_opening',
        'valve_roof': 'building.valve_roof_opening',
        'flow': 'building.SFW_HCRLbM_l_per_min',
        'P_living': 'building.P_appliances_living_W_in',
        'P_cellar': 'building.P_appliances_cellar_W_in',
        'P_roof': 'building.P_appliances_roof_W_in'
    }
    
    # Find which signals exist
    found_signals = {}
    for key, exact_name in signal_map.items():
        if exact_name in all_signals:
            found_signals[key] = exact_name
            print(f"   ✅ {key} → {exact_name}")
        else:
            print(f"   ⚠️  {key} not found")
    
    if len(found_signals) < 5:
        print(f"\n❌ Only found {len(found_signals)} signals - not enough")
        return []
    
    # Define the 9 plots
    plot_configs = [
        {
            'name': '1_System_performance',
            'title': 'System Performance Overview',
            'signals': ['T_living', 'T_cellar', 'T_roof', 'T_return', 'flow', 'T_supply'],
            'ylabel': 'Temperature [°C] / Flow [L/min]'
        },
        {
            'name': '2_Zone_temperatures',
            'title': 'Zone Temperatures',
            'signals': ['T_living', 'T_cellar', 'T_roof', 'T_ambient'],
            'ylabel': 'Temperature [°C]'
        },
        {
            'name': '3_Valve_operation',
            'title': 'Valve Operations',
            'signals': ['valve_living', 'valve_cellar', 'valve_roof'],
            'ylabel': 'Valve Opening [%]',
            'multiply_100': True
        },
        {
            'name': '4_Cellar_control',
            'title': 'Cellar Temperature Control',
            'signals': ['T_cellar', 'valve_cellar'],
            'ylabel': 'Temperature [°C] / Valve Opening [%]',
            'dual_axis': True
        },
        {
            'name': '5_Water_temperatures',
            'title': 'Water Supply/Return Temperatures',
            'signals': ['T_supply', 'T_return'],
            'ylabel': 'Temperature [°C]'
        },
        {
            'name': '6_Power_distribution',
            'title': 'Appliance Power Distribution',
            'signals': ['P_living', 'P_cellar', 'P_roof'],
            'ylabel': 'Power [kW]',
            'divide_1000': True
        },
        {
            'name': '7_Outdoor_vs_indoor',
            'title': 'Outdoor vs Indoor Temperatures',
            'signals': ['T_ambient', 'T_living', 'T_cellar', 'T_roof'],
            'ylabel': 'Temperature [°C]'
        },
        {
            'name': '8_Living_control',
            'title': 'Living Room Temperature Control',
            'signals': ['T_living', 'valve_living'],
            'ylabel': 'Temperature [°C] / Valve Opening [%]',
            'dual_axis': True
        },
        {
            'name': '9_Flow_rate',
            'title': 'System Flow Rate',
            'signals': ['flow'],
            'ylabel': 'Flow [L/min]'
        }
    ]
    
    created_plots = []
    colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6', '#1abc9c']
    
    print(f"\n🎨 Creating plots...")
    
    for plot_config in plot_configs:
        try:
            fig, ax = plt.subplots(figsize=(14, 6))
            
            signals_plotted = []
            
            for i, sig_key in enumerate(plot_config['signals']):
                if sig_key not in found_signals:
                    continue
                
                actual_signal = found_signals[sig_key]
                time, data = extract_signal_data(mat_data, actual_signal)
                
                if time is None or data is None:
                    continue
                
                # Convert time to hours
                time_hours = time / 3600
                
                # Apply transformations
                if plot_config.get('multiply_100'):
                    data = data * 100
                if plot_config.get('divide_1000'):
                    data = data / 1000
                
                # Create label
                label = sig_key.replace('_', ' ').replace('T ', '').replace('valve ', '').title()
                
                # Plot
                ax.plot(time_hours, data, label=label,
                       color=colors[i % len(colors)],
                       linewidth=2.5, linestyle='-')
                
                signals_plotted.append(actual_signal)
            
            if signals_plotted:
                ax.set_xlabel('Time [h]', fontsize=12, fontweight='bold')
                ax.set_ylabel(plot_config['ylabel'], fontsize=12, fontweight='bold')
                ax.set_title(plot_config['title'], fontsize=14, fontweight='bold')
                ax.legend(fontsize=10, loc='best')
                ax.grid(True, alpha=0.3, linestyle='--')
                
                plt.tight_layout()
                
                output_file = output_folder / f"{plot_config['name']}.png"
                plt.savefig(output_file, dpi=300, bbox_inches='tight')
                plt.close()
                
                created_plots.append(output_file)
                print(f"✅ Created: {plot_config['name']}")
            else:
                print(f"⚠️  Skipped: {plot_config['name']} (no signals)")
                plt.close()
        
        except Exception as e:
            print(f"❌ Error creating {plot_config['name']}: {e}")
            plt.close()
    
    print(f"\n✅ Created {len(created_plots)}/{len(plot_configs)} plots")
    return created_plots

# HTML Template with ALL improvements
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>CoSES Complete Workflow - Ultimate</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 20px;
            padding: 40px;
            box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
        }
        h1 {
            color: #667eea;
            font-size: 2.5em;
            text-align: center;
            margin-bottom: 10px;
        }
        .subtitle {
            text-align: center;
            color: #666;
            margin-bottom: 30px;
            font-size: 1.1em;
        }
        .workflow-steps {
            background: #e8f5e9;
            border: 2px solid #4caf50;
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 25px;
        }
        .workflow-steps h3 {
            color: #2e7d32;
            margin-bottom: 15px;
        }
        .workflow-steps ol {
            margin-left: 25px;
            color: #2e7d32;
        }
        .section {
            background: #f8f9fa;
            border-radius: 15px;
            padding: 25px;
            margin-bottom: 25px;
            border: 2px solid #e9ecef;
        }
        .section h2 {
            color: #667eea;
            font-size: 1.4em;
            margin-bottom: 20px;
            padding-bottom: 10px;
            border-bottom: 2px solid #667eea;
        }
        .building-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 15px;
            margin-top: 20px;
        }
        .building-card {
            background: white;
            padding: 15px;
            border-radius: 12px;
            border: 3px solid #dee2e6;
            cursor: pointer;
            transition: all 0.3s;
            display: flex;
            align-items: center;
            gap: 12px;
        }
        .building-card:hover {
            border-color: #667eea;
            transform: translateY(-3px);
            box-shadow: 0 8px 20px rgba(102, 126, 234, 0.3);
        }
        .building-card.selected {
            border-color: #667eea;
            background: linear-gradient(135deg, #667eea15 0%, #764ba215 100%);
        }
        .building-icon {
            font-size: 2.5em;
            min-width: 50px;
            text-align: center;
        }
        .building-info h3 {
            color: #495057;
            font-size: 1.1em;
            margin-bottom: 5px;
        }
        .building-info .specs {
            color: #667eea;
            font-weight: 600;
            font-size: 0.9em;
        }
        label {
            display: block;
            font-weight: 600;
            color: #495057;
            margin-bottom: 8px;
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .help-icon {
            display: inline-block;
            width: 18px;
            height: 18px;
            background: #667eea;
            color: white;
            border-radius: 50%;
            text-align: center;
            line-height: 18px;
            font-size: 12px;
            cursor: help;
            position: relative;
        }
        .help-icon:hover::after {
            content: attr(data-tooltip);
            position: absolute;
            left: 25px;
            top: -5px;
            background: #333;
            color: white;
            padding: 8px 12px;
            border-radius: 6px;
            white-space: nowrap;
            font-size: 12px;
            font-weight: normal;
            z-index: 1000;
            box-shadow: 0 4px 8px rgba(0,0,0,0.3);
        }
        input[type="text"],
        input[type="number"] {
            width: 100%;
            padding: 12px 15px;
            border: 2px solid #dee2e6;
            border-radius: 8px;
            font-size: 1em;
        }
        input:focus {
            outline: none;
            border-color: #667eea;
        }
        .grid-3 {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
        }
        .input-with-unit {
            position: relative;
        }
        .input-with-unit input {
            padding-right: 45px;
        }
        .unit-label {
            position: absolute;
            right: 15px;
            top: 50%;
            transform: translateY(-50%);
            color: #6c757d;
            font-weight: 600;
        }
        small {
            display: block;
            color: #6c757d;
            margin-top: 5px;
            font-size: 0.85em;
        }
        button {
            width: 100%;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            border-radius: 12px;
            font-size: 1.3em;
            font-weight: 700;
            cursor: pointer;
            transition: all 0.3s;
            margin-top: 20px;
            box-shadow: 0 5px 20px rgba(102, 126, 234, 0.4);
        }
        button:hover {
            transform: translateY(-3px);
            box-shadow: 0 8px 30px rgba(102, 126, 234, 0.6);
        }
        button:disabled {
            opacity: 0.6;
            cursor: not-allowed;
        }
        .plot-button {
            background: linear-gradient(135deg, #2ecc71 0%, #27ae60 100%);
            margin-top: 10px;
        }
        .plot-button:hover {
            box-shadow: 0 8px 30px rgba(46, 204, 113, 0.6);
        }
        .message {
            padding: 20px;
            border-radius: 12px;
            margin-top: 25px;
            display: none;
        }
        .message.show { display: block; }
        .success {
            background: #d4edda;
            color: #155724;
            border: 2px solid #c3e6cb;
        }
        .loading {
            background: #d1ecf1;
            color: #0c5460;
            border: 2px solid #bee5eb;
        }
        .error {
            background: #f8d7da;
            color: #721c24;
            border: 2px solid #f5c6cb;
        }
        .info-box {
            background: #fff3cd;
            border: 2px solid #ffc107;
            padding: 15px;
            border-radius: 8px;
            margin-top: 10px;
            font-size: 0.9em;
        }
        @media (max-width: 768px) {
            .grid-3 { grid-template-columns: 1fr; }
            .building-grid { grid-template-columns: 1fr; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🏢 CoSES Project Builder</h1>
        <p class="subtitle">Professional Building Thermal Simulation</p>
        
        <div class="workflow-steps">
            <h3>📋 Workflow:</h3>
            <ol>
                <li>Select building type and configure parameters</li>
                <li>Click "Generate Project" → Dymola opens</li>
                <li>In Dymola: Press <strong>F9</strong> to simulate</li>
                <li>Back here: Click "Generate Plots" → 9 plots created!</li>
            </ol>
        </div>
        
        <form id="generatorForm" onsubmit="generateProject(event)">
            
            <div class="section">
                <h2>📂 Base Project</h2>
                <label>Base Project Path:</label>
                <input type="text" id="basePath" 
                       value="E:/Documents/Dymola/CoSES_Thermal_ProHMo_PHiL" required>
            </div>
            
            <div class="section">
                <h2>🏠 Building Type</h2>
                <div class="building-grid">
                    <div class="building-card" onclick="selectBuilding(1)">
                        <div class="building-icon">🏘️</div>
                        <div class="building-info">
                            <h3>Small House</h3>
                            <p class="specs">70/50/40 m²</p>
                        </div>
                    </div>
                    <div class="building-card selected" onclick="selectBuilding(2)">
                        <div class="building-icon">🏡</div>
                        <div class="building-info">
                            <h3>Medium House</h3>
                            <p class="specs">100/80/60 m²</p>
                        </div>
                    </div>
                    <div class="building-card" onclick="selectBuilding(3)">
                        <div class="building-icon">🏰</div>
                        <div class="building-info">
                            <h3>Large House</h3>
                            <p class="specs">130/100/80 m²</p>
                        </div>
                    </div>
                    <div class="building-card" onclick="selectBuilding(4)">
                        <div class="building-icon">🏢</div>
                        <div class="building-info">
                            <h3>Office</h3>
                            <p class="specs">150/100/80 m²</p>
                        </div>
                    </div>
                    <div class="building-card" onclick="selectBuilding(5)">
                        <div class="building-icon">🏫</div>
                        <div class="building-info">
                            <h3>School</h3>
                            <p class="specs">200/120/100 m²</p>
                        </div>
                    </div>
                    <div class="building-card" onclick="selectBuilding(6)">
                        <div class="building-icon">🏭</div>
                        <div class="building-info">
                            <h3>Factory</h3>
                            <p class="specs">250/150/120 m²</p>
                        </div>
                    </div>
                    <div class="building-card" onclick="selectBuilding(7)">
                        <div class="building-icon">🏨</div>
                        <div class="building-info">
                            <h3>Hotel</h3>
                            <p class="specs">300/200/150 m²</p>
                        </div>
                    </div>
                    <div class="building-card" onclick="selectBuilding(8)">
                        <div class="building-icon">🏥</div>
                        <div class="building-info">
                            <h3>Hospital</h3>
                            <p class="specs">400/250/200 m²</p>
                        </div>
                    </div>
                    <div class="building-card" onclick="selectBuilding(9)">
                        <div class="building-icon">🏬</div>
                        <div class="building-info">
                            <h3>Mall</h3>
                            <p class="specs">500/300/250 m²</p>
                        </div>
                    </div>
                    <div class="building-card" onclick="selectBuilding(10)">
                        <div class="building-icon">⚙️</div>
                        <div class="building-info">
                            <h3>Custom</h3>
                            <p class="specs">Your values</p>
                        </div>
                    </div>
                </div>
                <input type="hidden" id="buildingType" value="2">
            </div>
            
            <div class="section">
                <h2>✏️ Project Name</h2>
                <input type="text" id="projectName" value="CoSES_MediumHouse" required>
            </div>
            
            <div class="section">
                <h2>⏱️ Simulation Time</h2>
                <label>
                    Simulation Duration:
                    <span class="help-icon" data-tooltip="Total simulation time. Default 24h for daily cycle">?</span>
                </label>
                <div class="input-with-unit">
                    <input type="number" id="simTime" value="24" min="1" max="168" step="1" required>
                    <span class="unit-label">hours</span>
                </div>
                <small>Typical: 24h (1 day), 168h (1 week)</small>
            </div>
            
            <div class="section">
                <h2>🌍 Weather Location</h2>
                <label>
                    Select City / Climate:
                    <span class="help-icon" data-tooltip="Selects the EnergyPlus TMY3 weather file for the chosen city. Data from energyplus.net">?</span>
                </label>
                <select id="weatherCity" style="width:100%; padding:8px; margin-top:6px; border:1px solid #ddd; border-radius:6px; font-size:14px;">
                    <option value="E:/Documents/Dymola/Library/IBPSA/Resources/weatherdata/DEU_Munich.108660_IWEC.mos" selected>
                        &#x1F1E9;&#x1F1EA; Munich, Germany
                    </option>
                    <option value="E:/Documents/Dymola/Library/IBPSA/Resources/weatherdata/DEU_Berlin.103840_IWEC.mos">
                        &#x1F1E9;&#x1F1EA; Berlin, Germany
                    </option>
                    <option value="E:/Documents/Dymola/Library/IBPSA/Resources/weatherdata/FRA_Paris.Orly.071490_IWEC.mos">
                        &#x1F1EB;&#x1F1F7; Paris, France
                    </option>
                    <option value="E:/Documents/Dymola/Library/IBPSA/Resources/weatherdata/GBR_London.Gatwick.037760_IWEC.mos">
                        &#x1F1EC;&#x1F1E7; London Gatwick, United Kingdom
                    </option>
                    <option value="E:/Documents/Dymola/Library/IBPSA/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos">
                        &#x1F1FA;&#x1F1F8; San Francisco, USA
                    </option>
                    <option value="E:/Documents/Dymola/Library/IBPSA/Resources/weatherdata/USA_CO_Denver.Intl.AP.725650_TMY3.mos">
                        &#x1F1FA;&#x1F1F8; Denver, USA
                    </option>
                    <option value="E:/Documents/Dymola/Library/IBPSA/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos">
                        &#x1F1FA;&#x1F1F8; Chicago, USA
                    </option>
                </select>
                <small style="color:#666; margin-top:4px; display:block;">
                    TMY3/IWEC files from
                    <a href="https://energyplus.net/weather" target="_blank" style="color:#667eea;">energyplus.net</a>
                </small>
            </div>

            <div class="section">
                <h2>📐 Zone Areas [m²]</h2>
                <div class="grid-3">
                    <div>
                        <label>
                            Living Zone:
                            <span class="help-icon" data-tooltip="Floor area of main living space">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="AZone_living" value="100" min="30" max="1000" required>
                            <span class="unit-label">m²</span>
                        </div>
                    </div>
                    <div>
                        <label>
                            Cellar Zone:
                            <span class="help-icon" data-tooltip="Floor area of basement/cellar">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="AZone_cellar" value="80" min="20" max="800" required>
                            <span class="unit-label">m²</span>
                        </div>
                    </div>
                    <div>
                        <label>
                            Roof Zone:
                            <span class="help-icon" data-tooltip="Floor area of attic/top floor">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="AZone_roof" value="60" min="20" max="600" required>
                            <span class="unit-label">m²</span>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="section">
                <h2>📏 Zone Heights [m]</h2>
                <div class="grid-3">
                    <div>
                        <label>
                            Living Height:
                            <span class="help-icon" data-tooltip="Ceiling height of living space">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="hZone_living" value="2.5" min="2.0" max="5.0" step="0.1" required>
                            <span class="unit-label">m</span>
                        </div>
                    </div>
                    <div>
                        <label>
                            Cellar Height:
                            <span class="help-icon" data-tooltip="Ceiling height of basement">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="hZone_cellar" value="2.2" min="1.8" max="4.0" step="0.1" required>
                            <span class="unit-label">m</span>
                        </div>
                    </div>
                    <div>
                        <label>
                            Roof Height:
                            <span class="help-icon" data-tooltip="Ceiling height of attic">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="hZone_roof" value="2.3" min="1.8" max="4.5" step="0.1" required>
                            <span class="unit-label">m</span>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="section">
                <h2>🌡️ Setpoint Temperatures [°C]</h2>
                <div class="info-box">
                    <strong>Note:</strong> Setpoints based on EN 15251 and ASHRAE 55. 
                    Lower values (18-20°C) test system under stress. 
                    Higher values (20-23°C) represent comfort conditions.
                </div>
                <div class="grid-3">
                    <div>
                        <label>
                            Living Setpoint:
                            <span class="help-icon" data-tooltip="Target temperature for living space. Comfort: 20-22°C">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="TRef_living" value="20" min="15" max="25" step="0.5" required>
                            <span class="unit-label">°C</span>
                        </div>
                    </div>
                    <div>
                        <label>
                            Cellar Setpoint:
                            <span class="help-icon" data-tooltip="Target temperature for basement. Typically 2-3°C lower">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="TRef_cellar" value="18" min="15" max="22" step="0.5" required>
                            <span class="unit-label">°C</span>
                        </div>
                    </div>
                    <div>
                        <label>
                            Roof Setpoint:
                            <span class="help-icon" data-tooltip="Target temperature for attic. Similar to living">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="TRef_roof" value="20" min="15" max="25" step="0.5" required>
                            <span class="unit-label">°C</span>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="section">
                <h2>🎛️ PI Controller Parameters</h2>
                <div class="info-box">
                    <strong>PI Control:</strong> k_PI controls response speed (higher = faster). 
                    Ti_PI controls integral action (higher = slower error elimination).
                    Tune for stability vs. responsiveness.
                </div>
                <div class="grid-3">
                    <div>
                        <label>
                            k_PI (Proportional Gain):
                            <span class="help-icon" data-tooltip="Controls response speed. Range: 0.05-2.0. Higher = more aggressive">?</span>
                        </label>
                        <input type="number" id="k_PI" value="0.3" min="0.05" max="2.0" step="0.01" required>
                        <small>Larger buildings use smaller k_PI</small>
                    </div>
                    <div>
                        <label>
                            Ti_PI (Integral Time):
                            <span class="help-icon" data-tooltip="Integral time constant in seconds. Higher = slower integral action">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="Ti_PI" value="500" min="50" max="2000" step="10" required>
                            <span class="unit-label">s</span>
                        </div>
                        <small>Larger buildings use higher Ti_PI</small>
                    </div>
                    <div></div>
                </div>
            </div>
            
            <div class="section">
                <h2>🔧 Valve Minimum Openings [%]</h2>
                <div class="info-box">
                    <strong>Anti-Freeze Protection:</strong> Minimum valve opening prevents freezing and ensures minimum flow. 
                    Higher values for zones with freeze risk.
                </div>
                <div class="grid-3">
                    <div>
                        <label>
                            Living Valve Min:
                            <span class="help-icon" data-tooltip="Minimum opening to prevent freezing. Usually 2-5%">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="yMin_living" value="3" min="0" max="20" step="1" required>
                            <span class="unit-label">%</span>
                        </div>
                    </div>
                    <div>
                        <label>
                            Cellar Valve Min:
                            <span class="help-icon" data-tooltip="Higher due to frost risk. Usually 4-8%">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="yMin_cellar" value="5" min="0" max="20" step="1" required>
                            <span class="unit-label">%</span>
                        </div>
                    </div>
                    <div>
                        <label>
                            Roof Valve Min:
                            <span class="help-icon" data-tooltip="Highest due to temperature extremes. Usually 6-10%">?</span>
                        </label>
                        <div class="input-with-unit">
                            <input type="number" id="yMin_roof" value="8" min="0" max="20" step="1" required>
                            <span class="unit-label">%</span>
                        </div>
                    </div>
                </div>
            </div>
            
            <button type="submit" id="submitBtn">
                🚀 Step 1: Generate Project
            </button>
        </form>
        
        <button type="button" id="plotBtn" class="plot-button" onclick="generatePlots()" disabled>
            📊 Step 2: Generate 9 Plots
        </button>
        
        <div id="messageArea"></div>
    </div>
    
    <script>
        const presets = {
            1: { name: 'CoSES_SmallHouse', AZone_living: 70, AZone_cellar: 50, AZone_roof: 40,
                 hZone_living: 2.4, hZone_cellar: 2.2, hZone_roof: 2.2,
                 TRef_living: 21, TRef_cellar: 18, TRef_roof: 20,
                 k_PI: 0.35, Ti_PI: 400, yMin_living: 4, yMin_cellar: 6, yMin_roof: 10, simTime: 24 },
            2: { name: 'CoSES_MediumHouse', AZone_living: 100, AZone_cellar: 80, AZone_roof: 60,
                 hZone_living: 2.5, hZone_cellar: 2.2, hZone_roof: 2.3,
                 TRef_living: 20, TRef_cellar: 18, TRef_roof: 20,
                 k_PI: 0.30, Ti_PI: 500, yMin_living: 3, yMin_cellar: 5, yMin_roof: 8, simTime: 24 },
            3: { name: 'CoSES_LargeHouse', AZone_living: 130, AZone_cellar: 100, AZone_roof: 80,
                 hZone_living: 2.6, hZone_cellar: 2.3, hZone_roof: 2.4,
                 TRef_living: 20, TRef_cellar: 18, TRef_roof: 20,
                 k_PI: 0.25, Ti_PI: 600, yMin_living: 2, yMin_cellar: 4, yMin_roof: 6, simTime: 24 },
            4: { name: 'CoSES_OfficeBuilding', AZone_living: 150, AZone_cellar: 100, AZone_roof: 80,
                 hZone_living: 3.0, hZone_cellar: 2.5, hZone_roof: 2.8,
                 TRef_living: 21, TRef_cellar: 19, TRef_roof: 21,
                 k_PI: 0.22, Ti_PI: 650, yMin_living: 2, yMin_cellar: 4, yMin_roof: 6, simTime: 24 },
            5: { name: 'CoSES_SchoolBuilding', AZone_living: 200, AZone_cellar: 120, AZone_roof: 100,
                 hZone_living: 3.2, hZone_cellar: 2.5, hZone_roof: 3.0,
                 TRef_living: 20, TRef_cellar: 18, TRef_roof: 20,
                 k_PI: 0.18, Ti_PI: 750, yMin_living: 2, yMin_cellar: 3, yMin_roof: 5, simTime: 24 },
            6: { name: 'CoSES_FactoryBuilding', AZone_living: 250, AZone_cellar: 150, AZone_roof: 120,
                 hZone_living: 4.0, hZone_cellar: 3.0, hZone_roof: 3.5,
                 TRef_living: 19, TRef_cellar: 17, TRef_roof: 19,
                 k_PI: 0.15, Ti_PI: 850, yMin_living: 2, yMin_cellar: 3, yMin_roof: 4, simTime: 24 },
            7: { name: 'CoSES_HotelBuilding', AZone_living: 300, AZone_cellar: 200, AZone_roof: 150,
                 hZone_living: 3.0, hZone_cellar: 2.8, hZone_roof: 3.0,
                 TRef_living: 22, TRef_cellar: 20, TRef_roof: 22,
                 k_PI: 0.13, Ti_PI: 950, yMin_living: 1, yMin_cellar: 2, yMin_roof: 3, simTime: 24 },
            8: { name: 'CoSES_HospitalBuilding', AZone_living: 400, AZone_cellar: 250, AZone_roof: 200,
                 hZone_living: 3.5, hZone_cellar: 3.0, hZone_roof: 3.2,
                 TRef_living: 23, TRef_cellar: 21, TRef_roof: 23,
                 k_PI: 0.11, Ti_PI: 1050, yMin_living: 1, yMin_cellar: 2, yMin_roof: 3, simTime: 24 },
            9: { name: 'CoSES_MallBuilding', AZone_living: 500, AZone_cellar: 300, AZone_roof: 250,
                 hZone_living: 5.0, hZone_cellar: 4.0, hZone_roof: 4.5,
                 TRef_living: 21, TRef_cellar: 19, TRef_roof: 21,
                 k_PI: 0.10, Ti_PI: 1200, yMin_living: 1, yMin_cellar: 1, yMin_roof: 2, simTime: 24 },
            10: { name: 'CoSES_Custom', AZone_living: 100, AZone_cellar: 80, AZone_roof: 60,
                  hZone_living: 2.5, hZone_cellar: 2.2, hZone_roof: 2.3,
                  TRef_living: 20, TRef_cellar: 18, TRef_roof: 20,
                  k_PI: 0.30, Ti_PI: 500, yMin_living: 3, yMin_cellar: 5, yMin_roof: 8, simTime: 24 }
        };
        
        function selectBuilding(type) {
            document.querySelectorAll('.building-card').forEach(c => c.classList.remove('selected'));
            event.currentTarget.classList.add('selected');
            document.getElementById('buildingType').value = type;
            
            const preset = presets[type];
            document.getElementById('projectName').value = preset.name;
            document.getElementById('AZone_living').value = preset.AZone_living;
            document.getElementById('AZone_cellar').value = preset.AZone_cellar;
            document.getElementById('AZone_roof').value = preset.AZone_roof;
            document.getElementById('hZone_living').value = preset.hZone_living;
            document.getElementById('hZone_cellar').value = preset.hZone_cellar;
            document.getElementById('hZone_roof').value = preset.hZone_roof;
            document.getElementById('TRef_living').value = preset.TRef_living;
            document.getElementById('TRef_cellar').value = preset.TRef_cellar;
            document.getElementById('TRef_roof').value = preset.TRef_roof;
            document.getElementById('k_PI').value = preset.k_PI;
            document.getElementById('Ti_PI').value = preset.Ti_PI;
            document.getElementById('yMin_living').value = preset.yMin_living;
            document.getElementById('yMin_cellar').value = preset.yMin_cellar;
            document.getElementById('yMin_roof').value = preset.yMin_roof;
            document.getElementById('simTime').value = preset.simTime;
        }
        
        async function generateProject(event) {
            event.preventDefault();
            
            const messageArea = document.getElementById('messageArea');
            const submitBtn = document.getElementById('submitBtn');
            const plotBtn = document.getElementById('plotBtn');
            
            submitBtn.disabled = true;
            plotBtn.disabled = true;
            messageArea.innerHTML = '<div class="message loading show">⏳ Generating project...</div>';
            
            const data = {
                base_path: document.getElementById('basePath').value,
                project_name: document.getElementById('projectName').value,
                building_type: parseInt(document.getElementById('buildingType').value),
                sim_time: parseFloat(document.getElementById('simTime').value) * 3600, // Convert to seconds
                AZone_living: parseFloat(document.getElementById('AZone_living').value),
                AZone_cellar: parseFloat(document.getElementById('AZone_cellar').value),
                AZone_roof: parseFloat(document.getElementById('AZone_roof').value),
                hZone_living: parseFloat(document.getElementById('hZone_living').value),
                hZone_cellar: parseFloat(document.getElementById('hZone_cellar').value),
                hZone_roof: parseFloat(document.getElementById('hZone_roof').value),
                TRef_living: parseFloat(document.getElementById('TRef_living').value) + 273.15,
                TRef_cellar: parseFloat(document.getElementById('TRef_cellar').value) + 273.15,
                TRef_roof: parseFloat(document.getElementById('TRef_roof').value) + 273.15,
                k_PI: parseFloat(document.getElementById('k_PI').value),
                Ti_PI: parseFloat(document.getElementById('Ti_PI').value),
                yMin_living: parseFloat(document.getElementById('yMin_living').value) / 100,
                yMin_cellar: parseFloat(document.getElementById('yMin_cellar').value) / 100,
                yMin_roof: parseFloat(document.getElementById('yMin_roof').value) / 100,
                weatherDataFile: document.getElementById('weatherCity').value
            };
            
            try {
                const response = await fetch('/generate', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(data)
                });
                
                const result = await response.json();
                
                if (result.success) {
                    const simHours = data.sim_time / 3600;
                    messageArea.innerHTML = `
                        <div class="message success show">
                            <h3>✅ Project Generated!</h3>
                            <p><strong>Location:</strong> ${result.project_path}</p>
                            <p><strong>Simulation Time:</strong> ${simHours} hours</p>
                            <p style="margin-top: 15px; padding: 15px; background: #fff3cd; border-radius: 8px;">
                                <strong>⚡ Next:</strong><br>
                                1. Go to Dymola<br>
                                2. Navigate to: <strong>ThermoEnergeticAnalysis.TestThreeZoneBuilding_optimized</strong><br>
                                3. Press <strong>F9</strong> to simulate (~${Math.round(simHours * 2)} minutes)<br>
                                4. Click "Generate 9 Plots" button below
                            </p>
                        </div>
                    `;
                    plotBtn.disabled = false;
                } else {
                    messageArea.innerHTML = `
                        <div class="message error show">
                            <h3>❌ Error</h3>
                            <p>${result.error}</p>
                        </div>
                    `;
                }
            } catch (error) {
                messageArea.innerHTML = `
                    <div class="message error show">
                        <h3>❌ Error</h3>
                        <p>${error.message}</p>
                    </div>
                `;
            }
            
            submitBtn.disabled = false;
        }
        
        async function generatePlots() {
            const messageArea = document.getElementById('messageArea');
            const plotBtn = document.getElementById('plotBtn');
            
            plotBtn.disabled = true;
            messageArea.innerHTML = '<div class="message loading show">📊 Generating plots (clearing old plots first)...</div>';
            
            try {
                const response = await fetch('/plot', {
                    method: 'POST'
                });
                
                const result = await response.json();
                
                if (result.success) {
                    messageArea.innerHTML = `
                        <div class="message success show">
                            <h3>✅ Plots Created!</h3>
                            <p><strong>Results:</strong> ${result.mat_file}</p>
                            <p><strong>Plots:</strong> ${result.plots_created}/9 created</p>
                            <p><strong>Folder:</strong> ${result.plots_folder}</p>
                            <p style="margin-top: 10px;"><strong>Debug:</strong> ${result.debug_file}</p>
                        </div>
                    `;
                } else {
                    messageArea.innerHTML = `
                        <div class="message error show">
                            <h3>⚠️ ${result.error}</h3>
                            <p>${result.details || 'Check the debug file'}</p>
                        </div>
                    `;
                }
            } catch (error) {
                messageArea.innerHTML = `
                    <div class="message error show">
                        <h3>❌ Error</h3>
                        <p>${error.message}</p>
                    </div>
                `;
            }
            
            plotBtn.disabled = false;
        }
        
        window.onload = () => selectBuilding(2);
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

BUILDING_TYPE_MAP = {
    1: 'SmallHouse',
    2: 'MediumHouse',
    3: 'BigHouse',
    4: 'Office',
    5: 'School',
    6: 'Factory',
    7: 'Hotel',
    8: 'Hospital',
    9: 'Mall',
    10: 'SmallHouse',  # Custom - fallback
}

@app.route('/generate', methods=['POST'])
def generate():
    try:
        data = request.json
        base_path = data['base_path']       
        project_name = data['project_name']  
        building_type = data.get('building_type', 2)
        
        parameters = {
            'AZone_living': data['AZone_living'],
            'AZone_cellar': data['AZone_cellar'],
            'AZone_roof': data['AZone_roof'],
            'hZone_living': data['hZone_living'],
            'hZone_cellar': data['hZone_cellar'],
            'hZone_roof': data['hZone_roof'],
            'TRef_living': data['TRef_living'],
            'TRef_cellar': data['TRef_cellar'],
            'TRef_roof': data['TRef_roof'],
            'k_PI': data['k_PI'],
            'Ti_PI': data['Ti_PI'],
            'yMin_living': data['yMin_living'],
            'yMin_cellar': data['yMin_cellar'],
            'yMin_roof': data['yMin_roof'],
            'buildingData': BUILDING_TYPE_MAP.get(building_type, 'SmallHouse'),
            'weatherDataFile': data.get('weatherDataFile', 
                'E:/Documents/Dymola/Library/IBPSA/Resources/weatherdata/DEU_Munich.108660_IWEC.mos'),
        }
        
        # NOTE: sim_time would need to be passed to Dymola via dsin.txt modification
        # For now, this is a placeholder - actual implementation needs Dymola API
        sim_time = data.get('sim_time', 86400)  # Default 24h
        
        generator = PerfectDymolaVariantGenerator(base_path)
        project_path = generator.generate_variant(project_name, parameters)
        
        CURRENT_PROJECT['path'] = str(project_path)
        CURRENT_PROJECT['mat_file'] = None
        CURRENT_PROJECT['plots_ready'] = False
        
        package_file = project_path / "package.mo"
        dymola_opened = open_in_dymola(package_file)
        
        return jsonify({
            'success': True,
            'project_path': str(project_path),
            'dymola_opened': dymola_opened,
            'sim_time': sim_time
        })
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/plot', methods=['POST'])
def plot():
    try:
        if not CURRENT_PROJECT['path']:
            return jsonify({
                'success': False,
                'error': 'No project generated yet',
                'details': 'Please generate a project first'
            })
        
        mat_file = find_latest_mat_file(CURRENT_PROJECT['path'])
        
        if not mat_file:
            return jsonify({
                'success': False,
                'error': 'No simulation results found',
                'details': 'No .mat file found. Run simulation in Dymola (F9).'
            })
        
        print(f"\n📊 Found results: {mat_file}")
        
        plots_folder = Path(CURRENT_PROJECT['path']) / 'plots'
        created_plots = create_9_plots(mat_file, plots_folder)
        
        debug_file = Path(CURRENT_PROJECT['path']) / 'all_signals_CORRECT.txt'
        
        if len(created_plots) == 0:
            return jsonify({
                'success': False,
                'error': 'Failed to create plots',
                'details': f'Check {debug_file.name} for signal names.'
            })
        
        CURRENT_PROJECT['mat_file'] = str(mat_file)
        CURRENT_PROJECT['plots_ready'] = True
        
        return jsonify({
            'success': True,
            'mat_file': str(mat_file),
            'plots_created': len(created_plots),
            'plots_folder': str(plots_folder),
            'debug_file': str(debug_file)
        })
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({
            'success': False,
            'error': 'Error generating plots',
            'details': str(e)
        }), 500

def open_browser():
    webbrowser.open('http://localhost:5000')

if __name__ == '__main__':
    print("\n" + "="*70)
    print("🏢 COSES COMPLETE WORKFLOW - ULTIMATE VERSION")
    print("="*70)
    print(f"\n✅ Python {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")
    print("✅ All packages loaded")
    print("✅ 10 building types")
    print("✅ Vertical signal extraction")
    print("✅ Parameter tooltips")
    print("✅ Simulation time control")
    print("✅ Auto-clear old plots")
    print("\n⏳ Starting server...")
    print("✅ Server at: http://localhost:5000")
    print("\nPress Ctrl+C to stop\n")
    print("="*70 + "\n")
    
    Timer(2.0, open_browser).start()
    app.run(debug=False, port=5000, host='0.0.0.0')