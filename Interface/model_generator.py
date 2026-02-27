import os
import shutil
import json
from pathlib import Path
from jinja2 import Template
import subprocess

class ModelicaProjectGenerator:
    """
    Generates complete Modelica/Dymola project from user parameters
    """
    
    def __init__(self, output_dir="generated_projects"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Load model templates
        self.templates = self.load_templates()
    
    def load_templates(self):
        """Load Modelica file templates with placeholders"""
        return {
            'building': self.get_building_template(),
            'phil': self.get_phil_template(),
            'test': self.get_test_template(),
            'hydronic': self.get_hydronic_template()
        }
    
    def generate_project(self, parameters):
        """
        Generate complete Dymola project
        
        Args:
            parameters: dict with all user inputs
        
        Returns:
            project_path: path to generated project folder
        """
        # Create project folder
        project_name = f"ThreeZone_{parameters['name']}"
        project_path = self.output_dir / project_name
        project_path.mkdir(exist_ok=True)
        
        # Generate each model file
        self._generate_building_model(project_path, parameters)
        self._generate_phil_model(project_path, parameters)
        self._generate_test_model(project_path, parameters)
        self._generate_hydronic_model(project_path, parameters)
        
        # Create package.mo
        self._generate_package_file(project_path, parameters)
        
        # Generate FMU if requested
        if parameters.get('generate_fmu', False):
            self._export_fmu(project_path, parameters)
        
        # Create documentation
        self._generate_readme(project_path, parameters)
        
        return project_path
    
    def _generate_building_model(self, project_path, params):
        """Generate ThreeZoneBuilding_optimized.mo"""
        template = Template(self.templates['building'])
        
        model_content = template.render(
            AZone_living=params['AZone_living'],
            AZone_cellar=params['AZone_cellar'],
            AZone_roof=params['AZone_roof'],
            hZone_living=params['hZone_living'],
            hZone_cellar=params['hZone_cellar'],
            hZone_roof=params['hZone_roof'],
            TRef_living=params['TRef_living'],
            TRef_cellar=params['TRef_cellar'],
            TRef_roof=params['TRef_roof'],
            k_PI=params['k_PI'],
            Ti_PI=params['Ti_PI'],
            yMin_living=params['yMin_living'],
            yMin_cellar=params['yMin_cellar'],
            yMin_roof=params['yMin_roof']
        )
        
        output_file = project_path / "ThreeZoneBuilding_optimized.mo"
        with open(output_file, 'w') as f:
            f.write(model_content)
    
    def _generate_phil_model(self, project_path, params):
        """Generate ThreeZoneBuilding_PHiL_optimized.mo"""
        template = Template(self.templates['phil'])
        
        model_content = template.render(
            # Parameters passed to PHiL wrapper
            **params
        )
        
        output_file = project_path / "ThreeZoneBuilding_PHiL_optimized.mo"
        with open(output_file, 'w') as f:
            f.write(model_content)
    
    def _export_fmu(self, project_path, params):
        """
        Export FMU using Dymola API
        Requires Dymola to be installed
        """
        try:
            # Import Dymola interface
            from dymola.dymola_interface import DymolaInterface
            
            # Start Dymola
            dymola = DymolaInterface()
            
            # Open project
            package_file = project_path / "package.mo"
            dymola.openModel(str(package_file))
            
            # Translate model
            model_name = f"{project_path.name}.ThreeZoneBuilding_PHiL_optimized"
            result = dymola.checkModel(model_name)
            
            if result:
                # Export FMU
                fmu_path = project_path / "ThreeZoneBuilding_PHiL_optimized.fmu"
                dymola.translateModelFMU(
                    model_name,
                    storeResult=False,
                    modelName="ThreeZoneBuilding_PHiL_optimized",
                    fmiVersion="2",
                    fmiType="cs"  # Co-simulation
                )
                
                print(f"✅ FMU generated: {fmu_path}")
            else:
                print("❌ Model check failed - FMU not generated")
            
            # Close Dymola
            dymola.close()
            
        except Exception as e:
            print(f"⚠️ FMU generation failed: {e}")
            print("Note: Dymola must be installed for automatic FMU export")
    
    def get_building_template(self):
        """
        Modelica template with placeholders for parameters
        """
        return '''
model ThreeZoneBuilding_optimized
  "Generated three-zone building model"
  
  // ═══════════════════════════════════════════════════════════
  // PARAMETERS - GENERATED FROM USER INPUTS
  // ═══════════════════════════════════════════════════════════
  
  // Zone geometry
  parameter Modelica.Units.SI.Area AZone_living={{ AZone_living }};
  parameter Modelica.Units.SI.Area AZone_cellar={{ AZone_cellar }};
  parameter Modelica.Units.SI.Area AZone_roof={{ AZone_roof }};
  
  parameter Modelica.Units.SI.Length hZone_living={{ hZone_living }};
  parameter Modelica.Units.SI.Length hZone_cellar={{ hZone_cellar }};
  parameter Modelica.Units.SI.Length hZone_roof={{ hZone_roof }};
  
  // Temperature setpoints
  parameter Modelica.Units.SI.Temperature TRef_living={{ TRef_living }};
  parameter Modelica.Units.SI.Temperature TRef_cellar={{ TRef_cellar }};
  parameter Modelica.Units.SI.Temperature TRef_roof={{ TRef_roof }};
  
  // Control parameters
  parameter Real k_PI={{ k_PI }};
  parameter Modelica.Units.SI.Time Ti_PI={{ Ti_PI }};
  parameter Real yMin_living={{ yMin_living }};
  parameter Real yMin_cellar={{ yMin_cellar }};
  parameter Real yMin_roof={{ yMin_roof }};
  
  // ... REST OF MODEL (insert complete model code) ...
  
end ThreeZoneBuilding_optimized;
'''
    
    # ... (more template methods) ...

# ═══════════════════════════════════════════════════════════
# WEB API (Flask)
# ═══════════════════════════════════════════════════════════

from flask import Flask, request, jsonify, send_file
import zipfile

app = Flask(__name__)
generator = ModelicaProjectGenerator()

@app.route('/')
def index():
    """Serve HTML interface"""
    return send_file('index.html')

@app.route('/generate', methods=['POST'])
def generate_project():
    """Generate project from user parameters"""
    try:
        # Get parameters from web form
        parameters = request.json
        
        # Validate inputs
        errors = validate_parameters(parameters)
        if errors:
            return jsonify({'success': False, 'errors': errors}), 400
        
        # Generate project
        project_path = generator.generate_project(parameters)
        
        # Create zip file for download
        zip_path = create_zip(project_path)
        
        return jsonify({
            'success': True,
            'message': 'Project generated successfully!',
            'download_url': f'/download/{zip_path.name}'
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/download/<filename>')
def download_project(filename):
    """Download generated project as zip"""
    file_path = generator.output_dir / filename
    return send_file(file_path, as_attachment=True)

def validate_parameters(params):
    """Validate user inputs"""
    errors = []
    
    # Check required parameters
    required = ['AZone_living', 'AZone_cellar', 'AZone_roof', 'k_PI', 'Ti_PI']
    for param in required:
        if param not in params:
            errors.append(f"Missing required parameter: {param}")
    
    # Check ranges
    if params.get('AZone_living', 0) < 50 or params.get('AZone_living', 0) > 200:
        errors.append("Living area must be between 50-200 m²")
    
    # ... more validation ...
    
    return errors

def create_zip(project_path):
    """Create zip file of project folder"""
    zip_path = project_path.parent / f"{project_path.name}.zip"
    
    with zipfile.ZipFile(zip_path, 'w') as zipf:
        for file in project_path.rglob('*'):
            if file.is_file():
                zipf.write(file, file.relative_to(project_path.parent))
    
    return zip_path

if __name__ == '__main__':
    app.run(debug=True, port=5000)