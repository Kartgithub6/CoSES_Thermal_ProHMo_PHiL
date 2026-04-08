within CoSES_Thermal_ProHMo_PHiL.Data;
record BaseBuilding "Base record for building parameters"
  extends Modelica.Icons.Record;

  // Zone Areas [m²]
  parameter Modelica.Units.SI.Area AZone_living "Living zone area";
  parameter Modelica.Units.SI.Area AZone_cellar "Cellar zone area";
  parameter Modelica.Units.SI.Area AZone_roof "Roof zone area";

  // Zone Heights [m]
  parameter Modelica.Units.SI.Length hZone_living "Living zone height";
  parameter Modelica.Units.SI.Length hZone_cellar "Cellar zone height";
  parameter Modelica.Units.SI.Length hZone_roof "Roof zone height";

  // Initial Temperatures [K]
  parameter Modelica.Units.SI.Temperature TZoneInit_living "Living initial temperature";
  parameter Modelica.Units.SI.Temperature TZoneInit_cellar "Cellar initial temperature";
  parameter Modelica.Units.SI.Temperature TZoneInit_roof "Roof initial temperature";

  // Reference Temperatures [K]
  parameter Modelica.Units.SI.Temperature TRef_living "Living reference temperature";
  parameter Modelica.Units.SI.Temperature TRef_cellar "Cellar reference temperature";
  parameter Modelica.Units.SI.Temperature TRef_roof "Roof reference temperature";

  // Control Parameters
  parameter Real k_PI "Proportional gain for PI controller";
  parameter Modelica.Units.SI.Time Ti_PI "Integral time for PI controller";
  parameter Real yMin_living "Minimum valve opening - living";
  parameter Real yMin_cellar "Minimum valve opening - cellar";
  parameter Real yMin_roof "Minimum valve opening - roof";

  // Heating Activation
  parameter Boolean cellarHeat "Enable heating in cellar";
  parameter Boolean roofHeat "Enable heating in roof";

  // Default Internal Loads [W]
  parameter Real P_appliances_living_default "Default appliance power - living";
  parameter Real P_appliances_cellar_default "Default appliance power - cellar";
  parameter Real P_appliances_roof_default "Default appliance power - roof";

  // Default Occupancy
  parameter Real nPersons_living_default "Default number of persons - living";
  parameter Real nPersons_cellar_default "Default number of persons - cellar";
  parameter Real nPersons_roof_default "Default number of persons - roof";

  // NEW: Thermal Resistance Parameters for FourElements Model
  // These parameters correspond to the thermal resistances in the FourElements RC model

  // Wall/Exterior Thermal Resistances
  parameter Real Rwall(unit="K/W") = 2.857
    "Thermal resistance of exterior wall [K/W] - typical value for well-insulated wall";

  parameter Real RExtRem(unit="K/W") = 0.1265
    "Thermal resistance of remaining exterior wall [K/W] - accounts for additional layers";

  // Floor Thermal Resistances
  parameter Real Rfloor(unit="K/W") = 3.333
    "Thermal resistance of floor [K/W] - typical value for insulated floor";

  parameter Real RFloorRem(unit="K/W") = 0.1265
    "Thermal resistance of remaining floor [K/W] - accounts for additional layers";

  // Roof Thermal Resistances
  parameter Real Rroof(unit="K/W") = 5.000
    "Thermal resistance of roof [K/W] - typical value for well-insulated roof";

  parameter Real RRoofRem(unit="K/W") = 0.1265
    "Thermal resistance of remaining roof [K/W] - accounts for additional layers";

  annotation(Documentation(info="<html>
<p><b>Base Record for Building Parameters - UPDATED</b></p>

<p>This record contains all building parameters for the three-zone building model.</p>
<p>This record is used as a template for specific building types.</p>

<li>
Dec 7, 2025, by Karthik Murugesan
</li>
<p><b>Thermal Resistance Parameters</b></p>
<p>Added thermal resistance parameters for the FourElements RC model:</p>
<ul>
<li><b>Wall Resistances:</b>
  <ul>
    <li>Rwall: Main wall thermal resistance [K/W]</li>
    <li>RExtRem: Remaining exterior thermal resistance [K/W]</li>
  </ul>
</li>
<li><b>Floor Resistances:</b>
  <ul>
    <li>Rfloor: Main floor thermal resistance [K/W]</li>
    <li>RFloorRem: Remaining floor thermal resistance [K/W]</li>
  </ul>
</li>
<li><b>Roof Resistances:</b>
  <ul>
    <li>Rroof: Main roof thermal resistance [K/W]</li>
    <li>RRoofRem: Remaining roof thermal resistance [K/W]</li>
  </ul>
</li>
</ul>

<li>
Oct 30, 2025, by Karthik Murugesan
</li>
<p><b>Thermal Resistance Guidelines:</b></p>
<ul>
<li>Higher R-values indicate better insulation</li>
<li>Typical ranges:
  <ul>
    <li>Poor insulation: R = 0.5-1.5 K/W</li>
    <li>Standard insulation: R = 1.5-3.0 K/W</li>
    <li>Good insulation: R = 3.0-5.0 K/W</li>
    <li>Excellent insulation: R > 5.0 K/W</li>
  </ul>
</li>
<li>RExtRem, RFloorRem, RRoofRem typically range from 0.05-0.20 K/W</li>
</ul>

<p><b>How to Calculate R-values:</b></p>
<p>R = 1 / (U × A)</p>
<p>Where:</p>
<ul>
<li>R = Thermal resistance [K/W]</li>
<li>U = U-value (thermal transmittance) [W/(m²·K)]</li>
<li>A = Area [m²]</li>
</ul>

<p><b>Example:</b></p>
<p>Wall with U-value = 0.35 W/(m²·K) and area = 100 m²:</p>
<p>R = 1 / (0.35 × 100) = 0.0286 K/W (for entire wall)</p>
<p>For a single reference element (e.g., 1 m²): R = 1 / 0.35 = 2.857 K/W</p>
</html>"));
end BaseBuilding;
