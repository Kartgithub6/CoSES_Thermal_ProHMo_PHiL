"""
PERFECT Dymola Project Variant Generator
- Asks for EVERY parameter individually
- Clear building type descriptions
- Custom project naming
- Fully working variants
"""

import re
import shutil
import stat
import os
from pathlib import Path
from datetime import datetime

class PerfectDymolaVariantGenerator:
    """
    Creates fully working Dymola project variants with custom names
    """
    
    def __init__(self, base_project_path, output_dir=None):
        self.base_project = Path(base_project_path)
        if not self.base_project.exists():
            raise ValueError(f"Project not found: {base_project_path}")
        
        self.base_name = self.base_project.name
        
        if output_dir:
            self.output_dir = Path(output_dir)
        else:
            self.output_dir = self.base_project.parent
        
        # Building presets with detailed descriptions
        self.presets = {
            '1': {
                'name': 'SmallHouse',
                'description': 'Small House - Compact apartment/small home',
                'AZone_living': 70, 
                'AZone_cellar': 50, 
                'AZone_roof': 40,
                'hZone_living': 2.4, 
                'hZone_cellar': 2.2, 
                'hZone_roof': 2.2,
                'TRef_living': 21,
                'TRef_cellar': 18,
                'TRef_roof': 20,
                'k_PI': 0.35, 
                'Ti_PI': 400,
                'yMin_living': 0.04, 
                'yMin_cellar': 0.06, 
                'yMin_roof': 0.10
            },
            '2': {
                'name': 'MediumHouse',
                'description': 'Medium House - Standard family home',
                'AZone_living': 100, 
                'AZone_cellar': 80, 
                'AZone_roof': 60,
                'hZone_living': 2.5, 
                'hZone_cellar': 2.2, 
                'hZone_roof': 2.3,
                'TRef_living': 20,
                'TRef_cellar': 18,
                'TRef_roof': 20,
                'k_PI': 0.3, 
                'Ti_PI': 500,
                'yMin_living': 0.03, 
                'yMin_cellar': 0.05, 
                'yMin_roof': 0.08
            },
            '3': {
                'name': 'LargeHouse',
                'description': 'Large House - Spacious luxury home',
                'AZone_living': 130, 
                'AZone_cellar': 100, 
                'AZone_roof': 80,
                'hZone_living': 2.6, 
                'hZone_cellar': 2.3, 
                'hZone_roof': 2.4,
                'TRef_living': 20,
                'TRef_cellar': 18,
                'TRef_roof': 20,
                'k_PI': 0.25, 
                'Ti_PI': 600,
                'yMin_living': 0.02, 
                'yMin_cellar': 0.04, 
                'yMin_roof': 0.06
            },
            '4': {
                'name': 'OfficeBuilding',
                'description': 'Office Building - Commercial workspace',
                'AZone_living': 150, 
                'AZone_cellar': 100, 
                'AZone_roof': 80,
                'hZone_living': 3.0, 
                'hZone_cellar': 2.5, 
                'hZone_roof': 2.8,
                'TRef_living': 21,
                'TRef_cellar': 19,
                'TRef_roof': 21,
                'k_PI': 0.25, 
                'Ti_PI': 600,
                'yMin_living': 0.02, 
                'yMin_cellar': 0.04, 
                'yMin_roof': 0.06
            },
            '5': {
                'name': 'SchoolBuilding',
                'description': 'School Building - Educational facility',
                'AZone_living': 200, 
                'AZone_cellar': 120, 
                'AZone_roof': 100,
                'hZone_living': 3.2, 
                'hZone_cellar': 2.5, 
                'hZone_roof': 3.0,
                'TRef_living': 20,
                'TRef_cellar': 18,
                'TRef_roof': 20,
                'k_PI': 0.2, 
                'Ti_PI': 700,
                'yMin_living': 0.02, 
                'yMin_cellar': 0.03, 
                'yMin_roof': 0.05
            },
            '6': {
                'name': 'Custom',
                'description': 'Custom - Enter all values manually',
                'AZone_living': 100, 
                'AZone_cellar': 80, 
                'AZone_roof': 60,
                'hZone_living': 2.5, 
                'hZone_cellar': 2.2, 
                'hZone_roof': 2.3,
                'TRef_living': 20,
                'TRef_cellar': 18,
                'TRef_roof': 20,
                'k_PI': 0.3, 
                'Ti_PI': 500,
                'yMin_living': 0.03, 
                'yMin_cellar': 0.05, 
                'yMin_roof': 0.08
            }
        }
    
    def generate_variant(self, project_name, parameters):
        """
        Generate complete working variant with custom project name
        
        Args:
            project_name: Custom name for the project
            parameters: Dict with parameter values
        
        Returns:
            Path to created project
        """
        print(f"\n{'='*70}")
        print(f"🏗️  CREATING PROJECT: {project_name}")
        print(f"{'='*70}\n")
        
        # Create variant folder with custom name
        variant_path = self.output_dir / project_name
        
        # Remove if exists
        if variant_path.exists():
            print(f"⚠️  Removing existing project: {project_name}")
            shutil.rmtree(variant_path, onerror=lambda f,p,e: (os.chmod(p, stat.S_IWRITE), f(p)))
        
        # Create backup of original
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        backup_path = self.output_dir / f"{self.base_name}_backup_{timestamp}"
        
        print(f"📦 Creating backup: {backup_path.name}")
        shutil.copytree(self.base_project, backup_path, ignore=shutil.ignore_patterns('.git'))
        
        # Copy to variant
        print(f"📁 Creating project: {project_name}")
        shutil.copytree(self.base_project, variant_path, ignore=shutil.ignore_patterns('.git'))
        
        # Fix package name to match custom project name
        print(f"\n🔧 Setting package name to: {project_name}")
        self._fix_package_name(variant_path, project_name)
        
        # Fix within clauses
        print(f"🔧 Fixing 'within' clauses...")
        self._fix_within_clauses(variant_path, project_name)
        
        # Modify parameters
        print(f"\n📝 Applying your parameter values...")
        self._modify_all_parameters(variant_path, parameters)
        
        print(f"\n{'='*70}")
        print(f"✅ PROJECT CREATED SUCCESSFULLY!")
        print(f"{'='*70}")
        print(f"\n📂 Location: {variant_path}")
        print(f"\n🚀 Open in Dymola:")
        print(f"   {variant_path / 'package.mo'}")
        print(f"\n💡 To simulate:")
        print(f"   1. Open Dymola")
        print(f"   2. File → Open → {variant_path / 'package.mo'}")
        print(f"   3. Navigate to: ThermoEnergeticAnalysis.TestThreeZoneBuilding_optimized")
        print(f"   4. Press F9 to simulate")
        print(f"\n{'='*70}\n")
        
        return variant_path
    
    def _fix_package_name(self, project_path, new_name):
        """Fix package name in package.mo"""
        package_file = project_path / "package.mo"
        
        if not package_file.exists():
            print("   ⚠️  package.mo not found")
            return
        
        with open(package_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Replace package declaration
        content = re.sub(
            rf'package\s+{re.escape(self.base_name)}\b',
            f'package {new_name}',
            content
        )
        
        # Replace end statement
        content = re.sub(
            rf'end\s+{re.escape(self.base_name)};',
            f'end {new_name};',
            content
        )
        
        with open(package_file, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print(f"   ✅ Package name updated in package.mo")
    
    def _fix_within_clauses(self, project_path, new_name):
        """Fix all 'within' clauses to match new package name"""
        
        mo_files = list(project_path.rglob('*.mo'))
        fixed_count = 0
        
        for mo_file in mo_files:
            if mo_file.name == 'package.mo':
                continue
            
            try:
                with open(mo_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                new_content = re.sub(
                    rf'within\s+{re.escape(self.base_name)}([\s;.])',
                    f'within {new_name}\\1',
                    content
                )
                
                if new_content != content:
                    with open(mo_file, 'w', encoding='utf-8') as f:
                        f.write(new_content)
                    fixed_count += 1
            except Exception as e:
                print(f"   ⚠️  Error fixing {mo_file.name}: {e}")
        
        print(f"   ✅ Fixed {fixed_count} 'within' clauses")
    
    def _modify_all_parameters(self, project_path, parameters):
        """Modify parameters in ALL relevant files"""

        files_to_modify = [
            # ── ThermoEnergeticAnalysis ──────────────────────────────────────
            "ThermoEnergeticAnalysis/ThreeZoneBuilding_optimized.mo",
            "ThermoEnergeticAnalysis/ThreeZoneBuilding_PHiL_optimized.mo",
            "ThermoEnergeticAnalysis/ThreeZoneBuilding_PHiL_optimized_withWeather.mo",
            # ── Examples ────────────────────────────────────────────────────
            "Examples/TestThreeZoneBuilding_optimized.mo",
            "Examples/TestThreeZoneBuilding_withWeather.mo",
            "Examples/TestThreeZoneBuilding_withWeather_ScenarioB.mo"
        ]

        # Also discover any other .mo files in Examples/ automatically
        examples_dir = project_path / "Examples"
        if examples_dir.exists():
            for mo_file in examples_dir.glob("*.mo"):
                rel = str(mo_file.relative_to(project_path)).replace("\\", "/")
                if rel not in files_to_modify:
                    files_to_modify.append(rel)

        for file_path in files_to_modify:
            full_path = project_path / file_path
            if full_path.exists():
                print(f"\n   📄 {file_path}")
                self._modify_file_parameters(full_path, parameters)
    
    def _modify_file_parameters(self, file_path, parameters):
        """Modify parameters in a specific file"""
        
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        original_content = content
        modified_params = []
        
        for param_name, param_value in parameters.items():

            # ── weatherDataFile ──────────────────────────────────────────────
            # Three cases handled in order; continue is at the END after all cases.
            if param_name == 'weatherDataFile':
                changed = False

                # Case 1: parameter declaration in main model file
                #   parameter String weatherDataFile = "..."
                pattern1 = r'(parameter\s+String\s+weatherDataFile\s*=\s*)"([^"]*)"'
                if re.search(pattern1, content):
                    content = re.sub(pattern1, rf'\g<1>"{param_value}"', content)
                    modified_params.append(param_name)
                    print(f"      ✅ weatherDataFile [parameter decl] → {param_value}")
                    changed = True

                # Case 2: inside component instantiation (already present as override)
                #   weatherDataFile = "..."
                pattern2 = r'(weatherDataFile\s*=\s*)"([^"]*)"'
                if re.search(pattern2, content):
                    content = re.sub(pattern2, rf'\g<1>"{param_value}"', content)
                    if param_name not in modified_params:
                        modified_params.append(param_name)
                    print(f"      ✅ weatherDataFile [component override] → {param_value}")
                    changed = True

                # Case 3: weatherDataFile not present → inject before closing ) of building(...)
                #   Used for ScenarioB which has no weatherDataFile override line.
                #   Finds the last TZoneInit_xxx = NNN) line and adds weatherDataFile after it.
                if not changed:
                    inject_pattern = r'([ \t]*)(TZoneInit_\w+\s*=\s*[\d.]+)([ \t]*\))'
                    if re.search(inject_pattern, content):
                        content = re.sub(
                            inject_pattern,
                            rf'\g<1>\g<2>,\n\g<1>weatherDataFile = "{param_value}"\g<3>',
                            content
                        )
                        modified_params.append(param_name)
                        print(f"      ✅ weatherDataFile [injected after TZoneInit] → {param_value}")
                        changed = True

                if not changed:
                    print(f"      ⚠️  weatherDataFile: no pattern matched in {file_path.name}")
                continue  # ← MUST be here, after all three cases

            # ── buildingData ─────────────────────────────────────────────────
            # Three syntaxes handled; continue is at the END after all syntaxes.
            if param_name == 'buildingData':
                changed = False

                # Syntax A: assignment in ThermoEnergeticAnalysis files
                #   buildingData = CoSES_X.Data.ClassName()
                match_a = re.search(
                    r'buildingData\s*=\s*([\w.]+\.)(\w+)\(\)',
                    content
                )
                if match_a:
                    prefix = match_a.group(1)   # e.g. "CoSES_X.Data."
                    content = re.sub(
                        r'(buildingData\s*=\s*)[\w.]+\(\)',
                        rf'\g<1>{prefix}{param_value}()',
                        content
                    )
                    modified_params.append(param_name)
                    print(f"      ✅ buildingData [assignment] → {prefix}{param_value}()")
                    changed = True

                # Syntax B: redeclare in Examples files
                #   redeclare CoSES_X.Data.ClassName buildingData
                match_b = re.search(
                    r'(redeclare\s+)([\w.]+\.)(\w+)(\s+buildingData)',
                    content
                )
                if match_b:
                    prefix = match_b.group(2)   # e.g. "CoSES_X.Data."
                    new_redecl = f'{match_b.group(1)}{prefix}{param_value}{match_b.group(4)}'
                    content = re.sub(
                        r'redeclare\s+[\w.]+\.\w+\s+buildingData',
                        new_redecl,
                        content
                    )
                    if param_name not in modified_params:
                        modified_params.append(param_name)
                    print(f"      ✅ buildingData [redeclare] → {prefix}{param_value}")
                    changed = True

                # Syntax C: replaceable parameter in main model file
                #   replaceable parameter CoSES_X.Data.ClassName buildingData
                match_c = re.search(
                    r'(replaceable\s+parameter\s+)([\w.]+\.)(\w+)(\s+buildingData)',
                    content
                )
                if match_c:
                    prefix = match_c.group(2)   # e.g. "CoSES_X.Data."
                    new_decl = f'{match_c.group(1)}{prefix}{param_value}{match_c.group(4)}'
                    content = re.sub(
                        r'replaceable\s+parameter\s+[\w.]+\.\w+\s+buildingData',
                        new_decl,
                        content
                    )
                    if param_name not in modified_params:
                        modified_params.append(param_name)
                    print(f"      ✅ buildingData [replaceable param] → {prefix}{param_value}")
                    changed = True

                if not changed:
                    print(f"      ⚠️  buildingData: no pattern matched in {file_path.name}")
                continue  # ← MUST be here, after all three syntaxes

            # ── Generic numeric parameters ────────────────────────────────────
            # Pattern 1: parameter declaration
            pattern1 = rf'(parameter\s+\S+\s+{re.escape(param_name)}\s*=\s*)([^;,\)]+)'
            if re.search(pattern1, content):
                content = re.sub(pattern1, rf'\g<1>{param_value}', content)
                modified_params.append(param_name)
                continue
            
            # Pattern 2: component instantiation parameter
            pattern2 = rf'(\b{re.escape(param_name)}\s*=\s*)([^;,\)]+)'
            if re.search(pattern2, content):
                content = re.sub(pattern2, rf'\g<1>{param_value}', content)
                modified_params.append(param_name)
        
        if content != original_content:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)
            for param in modified_params:
                if param not in ('buildingData', 'weatherDataFile'):
                    print(f"      ✅ {param} = {parameters[param]}")
        else:
            print(f"      ⚠️  No parameters modified (might be OK)")
    
    def interactive_mode(self):
        """Interactive terminal interface - asks for EVERY parameter"""
        
        print("\n" + "="*70)
        print("🏗️  PERFECT DYMOLA PROJECT VARIANT GENERATOR")
        print("="*70)
        print(f"\n📂 Base project: {self.base_project.name}")
        print(f"📂 Output directory: {self.output_dir}")
        
        # Select preset or custom
        print("\n" + "="*70)
        print("STEP 1: SELECT BUILDING TYPE (Initial Values)")
        print("="*70)
        for key, preset in self.presets.items():
            print(f"  {key}. {preset['description']}")
            print(f"     Areas: Living {preset['AZone_living']}m², "
                  f"Cellar {preset['AZone_cellar']}m², "
                  f"Roof {preset['AZone_roof']}m²")
        print("="*70)
        
        choice = input("\nSelect building type (1-6): ").strip()
        
        if choice not in self.presets:
            print("❌ Invalid choice!")
            return
        
        preset = self.presets[choice].copy()
        preset_description = preset.pop('description')
        preset_name = preset.pop('name')
        
        print(f"\n✅ Selected: {preset_description}")
        print("\nInitial preset values loaded.")
        print("You will now enter EVERY parameter individually.")
        
        # Get custom project name
        print("\n" + "="*70)
        print("STEP 2: PROJECT NAME")
        print("="*70)
        print("\nThis name will be used for:")
        print("  - Folder name")
        print("  - Package name in package.mo")
        print("  - All 'within' clauses")
        print("\nExamples: MySmallHouse, Office_Munich, Test_Building_V2")
        print("NOTE: Use underscores instead of spaces!")
        
        default_name = preset_name
        project_name = input(f"\nProject name (default: {default_name}): ").strip()
        
        if not project_name:
            project_name = default_name
        
        # Replace spaces with underscores
        project_name = project_name.replace(' ', '_')
        
        print(f"\n✅ Project name: {project_name}")
        
        # Get ALL parameters individually
        print("\n" + "="*70)
        print("STEP 3: ENTER ALL PARAMETERS")
        print("="*70)
        print("\nEnter each value (press Enter to keep preset value)")
        
        parameters = {}
        
        # ZONE AREAS
        print("\n" + "-"*70)
        print("ZONE AREAS [m²]")
        print("-"*70)
        
        val = input(f"Living room area (preset: {preset['AZone_living']} m²): ").strip()
        parameters['AZone_living'] = float(val) if val else preset['AZone_living']
        
        val = input(f"Cellar area (preset: {preset['AZone_cellar']} m²): ").strip()
        parameters['AZone_cellar'] = float(val) if val else preset['AZone_cellar']
        
        val = input(f"Roof area (preset: {preset['AZone_roof']} m²): ").strip()
        parameters['AZone_roof'] = float(val) if val else preset['AZone_roof']
        
        # ZONE HEIGHTS
        print("\n" + "-"*70)
        print("ZONE HEIGHTS [m]")
        print("-"*70)
        
        val = input(f"Living room height (preset: {preset['hZone_living']} m): ").strip()
        parameters['hZone_living'] = float(val) if val else preset['hZone_living']
        
        val = input(f"Cellar height (preset: {preset['hZone_cellar']} m): ").strip()
        parameters['hZone_cellar'] = float(val) if val else preset['hZone_cellar']
        
        val = input(f"Roof height (preset: {preset['hZone_roof']} m): ").strip()
        parameters['hZone_roof'] = float(val) if val else preset['hZone_roof']
        
        # TEMPERATURES (show in Celsius, store in Kelvin)
        print("\n" + "-"*70)
        print("SETPOINT TEMPERATURES [°C]")
        print("-"*70)
        
        val = input(f"Living room setpoint (preset: {preset['TRef_living']} °C): ").strip()
        temp_living_c = float(val) if val else preset['TRef_living']
        parameters['TRef_living'] = temp_living_c + 273.15  # Convert to Kelvin
        
        val = input(f"Cellar setpoint (preset: {preset['TRef_cellar']} °C): ").strip()
        temp_cellar_c = float(val) if val else preset['TRef_cellar']
        parameters['TRef_cellar'] = temp_cellar_c + 273.15
        
        val = input(f"Roof setpoint (preset: {preset['TRef_roof']} °C): ").strip()
        temp_roof_c = float(val) if val else preset['TRef_roof']
        parameters['TRef_roof'] = temp_roof_c + 273.15
        
        # CONTROL PARAMETERS
        print("\n" + "-"*70)
        print("CONTROL PARAMETERS")
        print("-"*70)
        
        val = input(f"PI controller gain k_PI (preset: {preset['k_PI']}): ").strip()
        parameters['k_PI'] = float(val) if val else preset['k_PI']
        
        val = input(f"PI time constant Ti_PI [s] (preset: {preset['Ti_PI']} s): ").strip()
        parameters['Ti_PI'] = float(val) if val else preset['Ti_PI']
        
        # VALVE MINIMUMS
        print("\n" + "-"*70)
        print("VALVE MINIMUM OPENINGS [0-1]")
        print("-"*70)
        
        val = input(f"Living valve minimum yMin (preset: {preset['yMin_living']}): ").strip()
        parameters['yMin_living'] = float(val) if val else preset['yMin_living']
        
        val = input(f"Cellar valve minimum yMin (preset: {preset['yMin_cellar']}): ").strip()
        parameters['yMin_cellar'] = float(val) if val else preset['yMin_cellar']
        
        val = input(f"Roof valve minimum yMin (preset: {preset['yMin_roof']}): ").strip()
        parameters['yMin_roof'] = float(val) if val else preset['yMin_roof']
        
        # SUMMARY
        print("\n" + "="*70)
        print("SUMMARY OF YOUR PARAMETERS")
        print("="*70)
        print(f"\nProject Name: {project_name}")
        print(f"\nZone Areas:")
        print(f"  Living: {parameters['AZone_living']} m²")
        print(f"  Cellar: {parameters['AZone_cellar']} m²")
        print(f"  Roof:   {parameters['AZone_roof']} m²")
        print(f"\nZone Heights:")
        print(f"  Living: {parameters['hZone_living']} m")
        print(f"  Cellar: {parameters['hZone_cellar']} m")
        print(f"  Roof:   {parameters['hZone_roof']} m")
        print(f"\nSetpoints:")
        print(f"  Living: {parameters['TRef_living']-273.15} °C ({parameters['TRef_living']} K)")
        print(f"  Cellar: {parameters['TRef_cellar']-273.15} °C ({parameters['TRef_cellar']} K)")
        print(f"  Roof:   {parameters['TRef_roof']-273.15} °C ({parameters['TRef_roof']} K)")
        print(f"\nControl:")
        print(f"  k_PI:  {parameters['k_PI']}")
        print(f"  Ti_PI: {parameters['Ti_PI']} s")
        print(f"\nValve Minimums:")
        print(f"  Living: {parameters['yMin_living']}")
        print(f"  Cellar: {parameters['yMin_cellar']}")
        print(f"  Roof:   {parameters['yMin_roof']}")
        print("="*70)
        
        confirm = input("\nProceed with project creation? (y/n): ").strip().lower()
        
        if confirm == 'y':
            self.generate_variant(project_name, parameters)
        else:
            print("\n❌ Cancelled. No project created.")

if __name__ == '__main__':
    import sys
    
    print("\n" + "="*70)
    print("🏗️  PERFECT DYMOLA VARIANT GENERATOR")
    print("="*70)
    print("\nThis tool creates working Dymola project variants.")
    print("You will be asked for EVERY parameter individually.")
    print("You can choose a custom project name.")
    print("\n" + "="*70)
    
    # Get base project path
    if len(sys.argv) > 1:
        base_project = sys.argv[1]
    else:
        print("\n📂 Enter path to your base Dymola project:")
        print("Example: E:/Documents/Dymola/CoSES_Thermal_ProHMo_PHiL")
        base_project = input("\nProject path: ").strip().strip('"')
    
    try:
        generator = PerfectDymolaVariantGenerator(base_project)
        generator.interactive_mode()
    except ValueError as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n❌ Cancelled by user.")
        sys.exit(0)