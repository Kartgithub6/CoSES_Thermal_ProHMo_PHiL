"""
CoSES Complete Workflow 
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

# Global cache for signal extraction (performance optimization)
_signal_cache = None
_mat_data_cache = None

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
            r"C:\Program Files\Dymola 2025x Refresh 1\bin64\Dymola.exe",
            r"C:\Program Files\Dymola 2025x\bin64\Dymola.exe",
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
    """
    Extract signal names using VERTICAL reading method
    WITH CACHING for performance
    """
    global _signal_cache, _mat_data_cache
    
    # Use cache if available
    if _mat_data_cache is mat_data and _signal_cache is not None:
        return _signal_cache
    
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
    
    # Cache for future use
    _signal_cache = signals
    _mat_data_cache = mat_data
    
    return signals

def extract_signal_data(mat_data, signal_name):
    """Extract time and data for a specific signal"""
    names = extract_signal_names_vertical(mat_data)
    
    if signal_name not in names:
        print(f"   ⚠️  Signal not found: {signal_name}")
        return None, None
    
    try:
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
    except Exception as e:
        print(f"   ❌ Error extracting {signal_name}: {e}")
        return None, None

def create_9_plots(mat_file, output_folder):
    """Create the 9 standard plots with ALL FIXES"""
    
    global _signal_cache, _mat_data_cache
    _signal_cache = None  # Clear cache
    _mat_data_cache = None
    
    print(f"\n{'='*70}")
    print(f"📊 CREATING 9 ANALYSIS PLOTS")
    print(f"={'='*70}")
    
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
    
    # Extract signals ONCE (with caching)
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
    
    # Define the 9 plots with FIXED labels
    plot_configs = [
        {
            'name': '1_System_performance',
            'title': 'System Performance Overview',
            'signals': [
                ('T_living', 'Living Temp'),
                ('T_cellar', 'Cellar Temp'),
                ('T_roof', 'Roof Temp'),
                ('T_return', 'Return Temp'),
                ('flow', 'Flow Rate'),
                ('T_supply', 'Supply Temp')
            ],
            'ylabel': 'Temperature [°C] / Flow [L/min]'
        },
        {
            'name': '2_Zone_temperatures',
            'title': 'Zone Temperatures',
            'signals': [
                ('T_living', 'Living'),
                ('T_cellar', 'Cellar'),
                ('T_roof', 'Roof'),
                ('T_ambient', 'Outdoor')
            ],
            'ylabel': 'Temperature [°C]'
        },
        {
            'name': '3_Valve_operation',
            'title': 'Valve Operations',
            'signals': [
                ('valve_living', 'Living Valve'),
                ('valve_cellar', 'Cellar Valve'),
                ('valve_roof', 'Roof Valve')
            ],
            'ylabel': 'Valve Opening [%]',
            'multiply_100': True
        },
        {
            'name': '4_Cellar_control',
            'title': 'Cellar Temperature Control',
            'signals': [
                ('T_cellar', 'Temperature'),
                ('valve_cellar', 'Valve Opening')
            ],
            'ylabel': 'Temperature [°C] / Valve [%]',
            'multiply_100': True
        },
        {
            'name': '5_Water_temperatures',
            'title': 'Water Supply/Return Temperatures',
            'signals': [
                ('T_supply', 'Supply'),
                ('T_return', 'Return')
            ],
            'ylabel': 'Temperature [°C]'
        },
        {
            'name': '6_Power_distribution',
            'title': 'Appliance Power Distribution',
            'signals': [
                ('P_living', 'Living'),
                ('P_cellar', 'Cellar'),
                ('P_roof', 'Roof')
            ],
            'ylabel': 'Power [kW]',
            'divide_1000': True
        },
        {
            'name': '7_Outdoor_vs_indoor',
            'title': 'Outdoor vs Indoor Temperatures',
            'signals': [
                ('T_ambient', 'Outdoor'),
                ('T_living', 'Living'),
                ('T_cellar', 'Cellar'),
                ('T_roof', 'Roof')
            ],
            'ylabel': 'Temperature [°C]'
        },
        {
            'name': '8_Living_control',
            'title': 'Living Room Temperature Control',
            'signals': [
                ('T_living', 'Temperature'),
                ('valve_living', 'Valve Opening')
            ],
            'ylabel': 'Temperature [°C] / Valve [%]',
            'multiply_100': True
        },
        {
            'name': '9_Flow_rate',
            'title': 'System Flow Rate',
            'signals': [
                ('flow', 'Flow Rate')
            ],
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
            
            # NEW: signals is now list of tuples (key, label)
            for i, sig_info in enumerate(plot_config['signals']):
                if isinstance(sig_info, tuple):
                    sig_key, label = sig_info
                else:
                    # Fallback for old format
                    sig_key = sig_info
                    label = sig_key.replace('_', ' ').title()
                
                if sig_key not in found_signals:
                    continue
                
                actual_signal = found_signals[sig_key]
                time, data = extract_signal_data(mat_data, actual_signal)
                
                if time is None or data is None:
                    print(f"   ⚠️  Failed to extract data for {sig_key}")
                    continue
                
                # Convert time to hours
                time_hours = time / 3600
                
                # Apply transformations
                if plot_config.get('multiply_100'):
                    data = data * 100
                if plot_config.get('divide_1000'):
                    data = data / 1000
                
                # Plot with proper label
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
                print(f"   ✅ Created: {plot_config['name']}")
            else:
                print(f"   ⚠️  Skipped: {plot_config['name']} (no signals available)")
                plt.close()
        
        except Exception as e:
            print(f"   ❌ Error creating {plot_config['name']}: {e}")
            import traceback
            traceback.print_exc()
            plt.close()
    
    print(f"\n✅ Created {len(created_plots)}/{len(plot_configs)} plots")
    
    # Clear cache after use
    _signal_cache = None
    _mat_data_cache = None
    
    return created_plots

# Use the same HTML template from ULTIMATE version
from CoSES_Complete_Workflow import HTML_TEMPLATE

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/generate', methods=['POST'])
def generate():
    try:
        data = request.json
        
        base_path = data['base_path']
        project_name = data['project_name']
        
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
            'weatherDataFile': data.get('weatherDataFile',
                'E:/Documents/Dymola/Library/IBPSA/Resources/weatherdata/DEU_Munich.108660_IWEC.mos'),
        }
        
        sim_time = data.get('sim_time', 86400)
        
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
    print("🏢 COSES COMPLETE WORKFLOW")
    print("="*70)
    print(f"\n✅ Python {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")
    print("✅ All packages loaded")
    print("✅ 10 building types")
    print("✅ Vertical signal extraction")
    print("✅ CACHED extraction (performance)")
    print("✅ Fixed duplicate legends")
    print("✅ Fixed missing flow plot")
    print("✅ Auto-clear old plots")
    print("\n⏳ Starting server...")
    print("✅ Server at: http://localhost:5000")
    print("\nPress Ctrl+C to stop\n")
    print("="*70 + "\n")
    
    Timer(2.0, open_browser).start()
    app.run(debug=False, port=5000, host='0.0.0.0')