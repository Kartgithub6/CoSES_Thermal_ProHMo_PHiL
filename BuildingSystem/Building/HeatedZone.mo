within CoSES_Thermal_ProHMo_PHiL.BuildingSystem.Building;
model HeatedZone
  import Modelica.Constants.pi;


  inner Modelica.Fluid.System system(
    allowFlowReversal = false,
    energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics   = Modelica.Fluid.Types.Dynamics.SteadyState);


  model InnerMass "Properties of an inner Mass V1.0"
    parameter Real V( quantity="Geometry.Volume", displayUnit="m3") = 6 "Inner additional volumes (e.g. inner walls)";
    parameter Real rho( quantity="Thermics.Density", displayUnit="kg/m3") = 1800 "Density";
    parameter Real cp( quantity="Thermics.SpecHeatCapacity", displayUnit="kJ/(kg.K)") = 920 "Specific heat capacity"; // 4177 or 920
    annotation(Icon(coordinateSystem(extent={{-100,50},{100,-50}})));
  end InnerMass;
protected
  InnerMass Masses[Bound](
    each cp  = cpMass1,
    each rho = rhoMass1,
    each V   = VMass1)
    "Inner masses model"
    annotation(Placement(transformation(extent={{-98,86},{-78,96}})));

public
  Boundary Boundaries[Bound](
    // 03 Dec 18.00 added
   // geometry
    ABound      = {ABound1,  ABound2,  ABound3},
    AWindow     = {AWindow1, AWindow2, AWindow3},
    AOthers     = {AOthers1, AOthers2, AOthers3},

    // material
    rhoBound    = {rhoBound1, rhoBound2, rhoBound3},
    cpBound     = {cpBound1,  cpBound2,  cpBound3},
    dBound      = {dBound1,   dBound2,   dBound3},

    // U-values and g-values
    gWindow     = {gWindow1,  gWindow2,  gWindow3},
    uBound      = {uBound1,   uBound2,   uBound3},
    uWindow     = {uWindow1,  uWindow2,  uWindow3},
    uOthers     = {uOthers1,  uOthers2,  uOthers3},

    // correction factors
    epsDirt     = {epsDirt1,  epsDirt2,  epsDirt3},
    epsShading  = {epsShading1, epsShading2, epsShading3},
    epsFrame    = {epsFrame1,  epsFrame2,  epsFrame3},

    // orientation
    NormalVector = {
      normalVector(alphaInclination1, alphaOrientation1),
      normalVector(alphaInclination2, alphaOrientation2),
      normalVector(alphaInclination3, alphaOrientation3)},
    alphaInc    = {alphaInclination1, alphaInclination2, alphaInclination3},
    alphaOr     = {alphaOrientation1, alphaOrientation2, alphaOrientation3},

    // absorption and convection coefficients
    alphaBound    = {alphaBound1, alphaBound2, alphaBound3},
    alphaBoundOut = {alphaBoundOut1, 0,          alphaBoundOut3},
    alphaBoundIn  = {alphaBoundIn1,  alphaBoundIn2, alphaBoundIn3},

    // depth and contact
    depthBound    = {0, depthBound2, 0},
    contactBound  = {contactBound1, contactBound2, contactBound3},
    groundContact = {groundContact1, groundContact2, groundContact3},

    lambdaBound   = {lambdaBound1, lambdaBound2, lambdaBound3}
    /* // 14 Dec 23.00 commented
    // 14 Dec 23.00 - Binding equations for input variables
    TBoundIn = fill(TZoneAct, Bound),
    TBoundOut = TAmbientBound*/

)   "Boundary model"
    annotation(Placement(transformation(extent={{-126,86},{-106,96}})));

  // ------------------------------------------------------------------
  // Boundary result variables needed by equations later in the model
  // (must be declared here so they exist when used)
  // ------------------------------------------------------------------
  Real TBoundIn[Bound](each start=293.15, quantity="Thermics.Temp", displayUnit="°C")
    "Inner temperature of boundary"
    annotation(Dialog(group="Temperature", tab="Results 1", __esi_showAs=ShowAs.Result));
  Real TBoundOut[Bound](each start=293.15, quantity="Thermics.Temp", displayUnit="°C")
    "Outer temperature of boundary"
    annotation(Dialog(group="Temperature", tab="Results 1", __esi_showAs=ShowAs.Result));

  Real QBoundIn[Bound](quantity="Basics.Power", displayUnit="kW")
    "Inner heat losses/gains due to Boundary"
    annotation(Dialog(group="Walls and other boundaries - Heating and Cooling",
      tab="Results 2", __esi_showAs=ShowAs.Result));

  Real QBoundChange[Bound](quantity="Basics.Power", displayUnit="kW")
    "Interchanged heat between outer and inner parts of Boundaries"
    annotation(Dialog(group="Walls and other boundaries - Heating and Cooling",
      tab="Results 2", __esi_showAs=ShowAs.Result));

  Real QBoundOut[Bound](quantity="Basics.Power", displayUnit="kW")
    "Outer heat losses/gains due to Boundary"
    annotation(Dialog(group="Walls and other boundaries - Heating and Cooling",
      tab="Results 2", __esi_showAs=ShowAs.Result));

  Real TGround[Bound](each start=283.15, quantity="Thermics.Temp", displayUnit="°C")
    "Ground temperature for ground-contact boundaries"
    annotation(Dialog(group="Temperature", tab="Results 1", __esi_showAs=ShowAs.Result));

  // part 2
public
  parameter Integer NumberZones = 3 "Number of Zones"
    annotation(Dialog(group="Zones", tab="Model Initialization"));

  parameter Integer ZoneIndex(
    min=1,
    max=MaxNumberZones,
    start=1) = 1
    "Zone Index (1 - Living, 2 - Cellar, 3 - Roof)"
  annotation(Dialog(group="Zones", tab="Model Initialization"));

protected
  parameter Integer MaxNumberZones=15 "Maximum number of zones"
    annotation(Dialog(group="Zones",tab="Model Initialization"));
  parameter Integer contactBound1 = 0
    "Boundary 1 connected to: '0' = Ambience, '1,2,3 …' = Zone"
    annotation(Dialog(group="Ambience", tab="Boundary 1 - Walls"));
  parameter Real ABound1(quantity="Geometry.Area", displayUnit="m²") = 4*30
    "Total surface area of equivalent walls (sum of 4 sides)"
    annotation(Dialog(group="Dimensions", tab="Boundary 1 - Walls"));
  parameter Real AWindow1(quantity="Geometry.Area", displayUnit="m²") = 4*5
    "Total window area in exterior walls"
    annotation(Dialog(group="Dimensions", tab="Boundary 1 - Walls"));
  parameter Real AOthers1(quantity="Geometry.Area", displayUnit="m²") = 0
    "Other surface area (doors, etc.)"
    annotation(Dialog(group="Dimensions", tab="Boundary 1 - Walls"));
  parameter Real dBound1(quantity="Geometry.Length", displayUnit="m") = 0.36
    "Average wall thickness"
    annotation(Dialog(group="Dimensions", tab="Boundary 1 - Walls"));
  parameter Real rhoBound1(quantity="Thermics.Density", displayUnit="kg/m³") = 1800
    "Average wall density (masonry)"
    annotation(Dialog(group="Material", tab="Boundary 1 - Walls"));
  parameter Real cpBound1(quantity="Thermics.SpecHeatCapacity", displayUnit="kJ/(kg·K)") = 920
    "Average wall specific heat capacity"
    annotation(Dialog(group="Material", tab="Boundary 1 - Walls"));
  parameter Real uBound1(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 0.20
    "Equivalent U-value for combined wall structure"
    annotation(Dialog(group="Heat Transmission", tab="Boundary 1 - Walls"));
  parameter Real uWindow1(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 1
    "Window U-value (double glazing)"
    annotation(Dialog(group="Heat Transmission", tab="Boundary 1 - Walls"));
  parameter Real uOthers1(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 0
    "Other surface transmission (unused)"
    annotation(Dialog(group="Heat Transmission", tab="Boundary 1 - Walls"));
  parameter Boolean groundContact1 = false if not contactBound1>0
    "Walls are not in ground contact"
    annotation(Dialog(group="Ground Position", tab="Boundary 1 - Walls"));
  parameter Real depthBound1(quantity="Geometry.Length", displayUnit="m") = 1
    if groundContact1 "Depth below ground if wall has ground contact"
    annotation(Dialog(group="Ground Position", tab="Boundary 1 - Walls"));
  parameter Real epsDirt1 = 0.1
    "Dirt correction for windows"
    annotation(Dialog(group="Correction Values", tab="Boundary 1 - Walls"));
  parameter Real epsShading1 = 0.2
    "Shading factor for walls"
    annotation(Dialog(group="Correction Values", tab="Boundary 1 - Walls"));
  parameter Real epsFrame1 = 0.2
    "Frame correction factor for windows"
    annotation(Dialog(group="Correction Values", tab="Boundary 1 - Walls"));
  parameter Real alphaInclination1(quantity="Geometry.Angle", displayUnit="°") = 1.5707963267948966
    "Average wall inclination angle (vertical)"
    annotation(Dialog(group="Alignment", tab="Boundary 1 - Walls"));
  parameter Real alphaOrientation1(quantity="Geometry.Angle", displayUnit="°") = 0
    "Orientation (not used for combined wall)"
    annotation(Dialog(group="Alignment", tab="Boundary 1 - Walls"));
  parameter Real gWindow1 = 0.6
    "Solar transmittance of glazing"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 1 - Walls"));
  parameter Real alphaBound1 = 0.2
    "Wall absorptance (light surface)"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 1 - Walls"));
  parameter Real alphaBoundOut1(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 20
    if not groundContact1 "Exterior convective coefficient (walls)"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 1 - Walls"));
  parameter Real alphaBoundIn1(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 7.5
    "Interior convective coefficient (walls)"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 1 - Walls"));
  parameter Integer contactBound2 = 0 if Bound > 1
    "Boundary 2 connected to: 0 = Ground, 1.. = Zone"
    annotation(Dialog(group="Ambience", tab="Boundary 2 - Floor"));
  parameter Real ABound2(quantity="Geometry.Area", displayUnit="m²") = 30 if Bound > 1
    "Floor surface area"
    annotation(Dialog(group="Dimensions", tab="Boundary 2 - Floor"));
  parameter Real AWindow2(quantity="Geometry.Area", displayUnit="m²") = 0 if Bound > 1
    "Window area (none for floor)"
    annotation(Dialog(group="Dimensions", tab="Boundary 2 - Floor"));
  parameter Real AOthers2(quantity="Geometry.Area", displayUnit="m²") = 0 if Bound > 1
    "Other floor surfaces"
    annotation(Dialog(group="Dimensions", tab="Boundary 2 - Floor"));
  parameter Real dBound2(quantity="Geometry.Length", displayUnit="m") = 0.36 if Bound > 1
    "Floor slab thickness"
    annotation(Dialog(group="Dimensions", tab="Boundary 2 - Floor"));
  parameter Real rhoBound2(quantity="Thermics.Density", displayUnit="kg/m³") = 1800 if Bound > 1
    "Floor density"
    annotation(Dialog(group="Material", tab="Boundary 2 - Floor"));
  parameter Real cpBound2(quantity="Thermics.SpecHeatCapacity", displayUnit="kJ/(kg·K)") = 920 if Bound > 1
    "Floor specific heat capacity"
    annotation(Dialog(group="Material", tab="Boundary 2 - Floor"));
  parameter Real uBound2(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 0.20 if Bound > 1
    "Floor U-value (ground contact)"
    annotation(Dialog(group="Heat Transmission", tab="Boundary 2 - Floor"));
  parameter Real uWindow2(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 0 if Bound > 1
    "Window U-value (unused)"
    annotation(Dialog(group="Heat Transmission", tab="Boundary 2 - Floor"));
  parameter Real uOthers2(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 0 if Bound > 1
    "Other surfaces U-value"
    annotation(Dialog(group="Heat Transmission", tab="Boundary 2 - Floor"));
  parameter Boolean groundContact2 = true if Bound > 1 and not contactBound2 > 0
    "Floor has ground contact"
    annotation(Dialog(group="Ground Position", tab="Boundary 2 - Floor"));
  parameter Real depthBound2(quantity="Geometry.Length", displayUnit="m") = 1.5
    if Bound > 1 and groundContact2
    "Depth below ground"
    annotation(Dialog(group="Ground Position", tab="Boundary 2 - Floor"));
  parameter Real epsDirt2 = 0 if Bound > 1
    "Dirt correction"
    annotation(Dialog(group="Correction Values", tab="Boundary 2 - Floor"));
  parameter Real epsShading2 = 1 if Bound > 1
    "Shading correction"
    annotation(Dialog(group="Correction Values", tab="Boundary 2 - Floor"));
  parameter Real epsFrame2 = 0 if Bound > 1
    "Frame correction"
    annotation(Dialog(group="Correction Values", tab="Boundary 2 - Floor"));
  parameter Real alphaInclination2(quantity="Geometry.Angle", displayUnit="°") = 0 if Bound > 1
    "Floor inclination"
    annotation(Dialog(group="Alignment", tab="Boundary 2 - Floor"));
  parameter Real alphaOrientation2(quantity="Geometry.Angle", displayUnit="°") = 0 if Bound > 1
    "Floor orientation"
    annotation(Dialog(group="Alignment", tab="Boundary 2 - Floor"));
  parameter Real gWindow2 = 0 if Bound > 1
    "Translucency (none)"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 2 - Floor"));
  parameter Real alphaBound2 = 0 if Bound > 1
    "Absorption coefficient"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 2 - Floor"));
  parameter Real alphaBoundOut2(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 20
    if Bound > 1 and not groundContact2
    "Outer convective coefficient"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 2 - Floor"));
  parameter Real alphaBoundIn2(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 7.5 if Bound > 1
    "Inner convective coefficient"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 2 - Floor"));
  parameter Integer contactBound3 = 0 if Bound > 2
    "Boundary 3 connected to: 0 = Ambience"
    annotation(Dialog(group="Ambience", tab="Boundary 3 - Roof"));
  parameter Real ABound3(quantity="Geometry.Area", displayUnit="m²") = 30 if Bound > 2
    "Roof surface area"
    annotation(Dialog(group="Dimensions", tab="Boundary 3 - Roof"));
  parameter Real AWindow3(quantity="Geometry.Area", displayUnit="m²") = 0 if Bound > 2
    "Roof window area"
    annotation(Dialog(group="Dimensions", tab="Boundary 3 - Roof"));
  parameter Real AOthers3(quantity="Geometry.Area", displayUnit="m²") = 0 if Bound > 2
    "Other roof surfaces"
    annotation(Dialog(group="Dimensions", tab="Boundary 3 - Roof"));
  parameter Real dBound3(quantity="Geometry.Length", displayUnit="m") = 0.36 if Bound > 2
    "Roof slab thickness"
    annotation(Dialog(group="Dimensions", tab="Boundary 3 - Roof"));
  parameter Real rhoBound3(quantity="Thermics.Density", displayUnit="kg/m³") = 1800 if Bound > 2
    "Roof density"
    annotation(Dialog(group="Material", tab="Boundary 3 - Roof"));
  parameter Real cpBound3(quantity="Thermics.SpecHeatCapacity", displayUnit="kJ/(kg·K)") = 920 if Bound > 2
    "Roof specific heat capacity"
    annotation(Dialog(group="Material", tab="Boundary 3 - Roof"));
  parameter Real uBound3(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 0.20 if Bound > 2
    "Roof U-value"
    annotation(Dialog(group="Heat Transmission", tab="Boundary 3 - Roof"));
  parameter Real uWindow3(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 0 if Bound > 2
    "Window U-value (unused)"
    annotation(Dialog(group="Heat Transmission", tab="Boundary 3 - Roof"));
  parameter Real uOthers3(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 0 if Bound > 2
    "Other U-value"
    annotation(Dialog(group="Heat Transmission", tab="Boundary 3 - Roof"));
  parameter Boolean groundContact3 = false if Bound > 2 and not contactBound3 > 0
    annotation(Dialog(group="Ground Position", tab="Boundary 3 - Roof"));

  parameter Real depthBound3(quantity="Geometry.Length", displayUnit="m") = 0
    if Bound > 2 and groundContact3
    "Depth (unused)"
    annotation(Dialog(group="Ground Position", tab="Boundary 3 - Roof"));
  parameter Real epsDirt3 = 0.1 if Bound > 2
    annotation(Dialog(group="Correction Values", tab="Boundary 3 - Roof"));
  parameter Real epsShading3 = 0.2 if Bound > 2
    annotation(Dialog(group="Correction Values", tab="Boundary 3 - Roof"));
  parameter Real epsFrame3 = 0.2 if Bound > 2
    annotation(Dialog(group="Correction Values", tab="Boundary 3 - Roof"));
  parameter Real alphaInclination3(quantity="Geometry.Angle", displayUnit="°") = 0 if Bound > 2
    "Floor inclination"
    annotation(Dialog(group="Alignment", tab="Boundary 3 - Roof"));
  parameter Real alphaOrientation3(quantity="Geometry.Angle", displayUnit="°") = 0 if Bound > 2
    "Floor orientation"
    annotation(Dialog(group="Alignment", tab="Boundary 3 - Roof"));
  parameter Real gWindow3 = 0 if Bound > 2
    "Translucency (none)"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 3 - Roof"));
  parameter Real alphaBound3 = 0 if Bound > 2
    "Absorption coefficient"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 3 - Roof"));
  parameter Real alphaBoundOut3(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 20
    if Bound > 2 and not groundContact3
    "Outer convective coefficient"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 3 - Roof"));
  parameter Real alphaBoundIn3(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 7.5 if Bound > 2
    "Inner convective coefficient"
    annotation(Dialog(group="Heat Absorption", tab="Boundary 3 - Roof"));
  parameter Modelica.Units.SI.Area ATransparentTot =
      AWindow1 + AWindow2 + AWindow3
    "Total transparent (window) area of all boundaries used in FourElements";
  parameter Modelica.Units.SI.Area ATot =
      ABound1 + ABound2 + ABound3
    "Total area of walls + floor + roof used in FourElements";
  parameter Boolean LoadCalculation = false
  "If enabled, only heating/cooling load calculation"
    annotation(Dialog(group="Calculation Mode", tab="Model Initialization"));

  parameter Real lambdaBound1 = max(1e-4,
    dBound1 / (1/uBound1 - 1/alphaBoundIn1 - 1/alphaBoundOut1));

  parameter Real lambdaBound2 = max(1e-4,
    dBound2 / (1/uBound2 - 1/alphaBoundIn2));

  parameter Real lambdaBound3 = max(1e-4,
    dBound3 / (1/uBound3 - 1/alphaBoundIn3 - 1/alphaBoundOut3));


public
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor
    infiltrationLoss(G=infiltrationRate*1.2*1005)
    "(1.2 kg/m³ = ρ_air, 1005 J/kg·K = cp_air) = infiltrationRate * rhoAir * cpAir"
    annotation (Placement(transformation(extent={{54,-6},{72,12}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    T_infiltration_air annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={102,0})));
  Modelica.Blocks.Sources.Constant T_outside(k=10  // 21 Dec FIX v11: Added default outdoor temp (10°C)
)   annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={142,0})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow occLoad
    "Occupany Load"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,66})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow appLoad
    "Appliance load"
    annotation (Placement(transformation(extent={{-10,54},{10,74}})));

public
  Modelica.Blocks.Interfaces.RealOutput TZone(quantity="ThermodynamicTemperature", unit="K", displayUnit="degC") "Zone temperature" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
  origin={-40,-102})));


  input Modelica.Blocks.Interfaces.RealInput TZoneRef(quantity="ThermodynamicTemperature", unit="K", displayUnit="degC") "Desired zone temperature"
  annotation (
    Placement(transformation(extent={{84,26},{112,54}})),
    iconTransformation(transformation(extent={{84,26},{112,54}})),
    Dialog(
      group="Others",
      tab="Results 1",
        __esi_showAs=ShowAs.Result));


  output Modelica.Blocks.Interfaces.RealOutput HeatCoolLoad(quantity="Power", unit="W", displayUnit="kW") "Heating/cooling power (positive = heating to zone)"
    annotation (
      Placement(
        transformation(
          origin={72,-103},
          extent = {{-10, -10}, {10, 10}},
          rotation = -90)),
      iconTransformation(
        transformation(
          origin = {0, -200},
          extent = {{-10, -10}, {10, 10}},
          rotation = -90)),
      Dialog(
        group = "Heating and Cooling Power",
        tab = "Results 1",
        __esi_showAs = ShowAs.Result));

  input Modelica.Blocks.Interfaces.RealInput WindowShading[Bound]
    "Vector for external shading of windows for each boundary (0=fully irradiated, 1=fully shaded)"
    annotation (
  Placement(transformation(extent={{-83,-119},{-123,-79}})),
        iconTransformation(
            transformation(extent={{370,-20},{330,20}})),
        Dialog(
            group="Others",
            tab="Results 1",
          __esi_showAs=ShowAs.Result));

  parameter Real kTZoneRef = 1e-9
  "Very small weight for TZoneRef in dynamic mode (to keep connector used)";

  // part 3
public
  parameter String DINFile =
    "C:/Users/Public/Documents/SimulationX 4.6/Modelica/GreenCity/Data/ModelData/building/DIN_factors/DIN_factors.txt";
  parameter String NumberPersonDINFile =
    "C:/Users/Public/Documents/SimulationX 4.6/Modelica/GreenCity/Data/ModelData/building/DIN_factors/DIN_persons.txt";
  parameter String PelDINFile =
    "C:/Users/Public/Documents/SimulationX 4.6/Modelica/GreenCity/Data/ModelData/building/DIN_factors/DIN_electrical.txt";
  parameter String InputDataFile =
    "C:/Users/Public/Documents/SimulationX 4.6/Modelica/GreenCity/Data/SimulationData/building/inner_loads/zone_x.txt";
  parameter Boolean DINcalc = false
  "If true, use DIN load calculation";
  parameter String CoolFactorTable      = "Si";
  parameter String LoadLightTable       = "m_light";
  parameter String LoadMachineTable     = "m_machine";
  parameter String NumberPersonDINTable = "person";
  parameter String PelDINTable          = "pel";
  parameter String NumberPersonTable    = "person";
  parameter String BaseLoadTable        = "baseLoad";
  parameter String NormLoadTable        = "normLoad";
  parameter String MachineLoadTable     = "machineLoad";
  parameter String LightLoadTable       = "lightLoad";
  parameter String InnerLoadTable       = "innerLoad";
  parameter String PelTable             = "pel";
  parameter String QelTable             = "qel";
  Modelica.Blocks.Tables.CombiTable1Dv CoolLoadFactorPerson(
      tableOnFile = DINcalc,
      table = [0, 0],
      tableName = CoolFactorTable,
      fileName = DINFile)
    if DINcalc annotation (Placement(transformation(extent={{184,-40},{
            204,-20}})));
  Modelica.Blocks.Tables.CombiTable1Dv CoolLoadFactorLigth(
      tableOnFile = DINcalc,
      table = [0, 0],
      tableName = CoolFactorTable,
      fileName = DINFile)
    if DINcalc annotation (Placement(transformation(extent={{186,-68},{
            206,-48}})));
  Modelica.Blocks.Tables.CombiTable1Dv CoolLoadFactorMachine(
      tableOnFile = DINcalc,
      table = [0, 0],
      tableName = CoolFactorTable,
      fileName = DINFile)
    if DINcalc annotation (Placement(transformation(extent={{184,-10},{
            204,10}})));
  Modelica.Blocks.Tables.CombiTable1Dv AppliedLoadFactorLight(
      tableOnFile = DINcalc,
      table = [0, 0],
      tableName = LoadLightTable,
      fileName = DINFile)
    if DINcalc annotation (Placement(transformation(extent={{182,22},{202,
            42}})));
  Modelica.Blocks.Tables.CombiTable1Dv AppliedLoadFactorMachine(
      tableOnFile = DINcalc,
      table = [0, 0],
      tableName = LoadMachineTable,
      fileName = DINFile)
    if DINcalc annotation (Placement(transformation(extent={{186,-94},{
            206,-74}})));
  Modelica.Blocks.Tables.CombiTable1Dv PelDIN(
      tableOnFile = DINcalc,
      table = [0, 0],
      tableName = PelDINTable,
      fileName = PelDINFile)
    if DINcalc annotation (Placement(transformation(extent={{220,-10},{
            240,10}})));
  Modelica.Blocks.Tables.CombiTable1Dv NumberPersonDIN(
      tableOnFile = DINcalc,
      table = [0, 0],
      tableName = NumberPersonDINTable,
      fileName = NumberPersonDINFile)
    if DINcalc annotation (Placement(transformation(extent={{-206,-6},{
            -186,14}})));
  Modelica.Blocks.Tables.CombiTable1Dv NumberPerson(
      tableOnFile = not DINcalc,
      table = [0, 0],
      tableName = NumberPersonTable,
      fileName = InputDataFile)
    if not DINcalc annotation (Placement(transformation(extent={{-206,-92},{-186,
            -72}})));

  Modelica.Blocks.Tables.CombiTable1Dv BaseLoad(
      tableOnFile = not DINcalc,
      table = [0, 0],
      tableName = BaseLoadTable,
      fileName = InputDataFile)
    if not DINcalc annotation (Placement(transformation(extent={{-206,24},
            {-186,44}})));
  Modelica.Blocks.Tables.CombiTable1Dv NormLoad(
      tableOnFile = not DINcalc,
      table = [0, 0],
      tableName = NormLoadTable,
      fileName = InputDataFile)
    if not DINcalc annotation (Placement(transformation(extent={{-206,54},{-186,
            74}})));
  Modelica.Blocks.Tables.CombiTable1Dv MachineLoad(
      tableOnFile = not DINcalc,
      table = [0, 0],
      tableName = MachineLoadTable,
      fileName = InputDataFile)
    if not DINcalc annotation (Placement(transformation(extent={{-206,82},{-186,
            102}})));
  Modelica.Blocks.Tables.CombiTable1Dv LightLoad(
      tableOnFile = not DINcalc,
      table = [0, 0],
      tableName = LightLoadTable,
      fileName = InputDataFile)
    if not DINcalc annotation (Placement(transformation(extent={{-206,-34},{-186,
            -14}})));
  Modelica.Blocks.Tables.CombiTable1Dv InnerLoad(
      tableOnFile = not DINcalc,
      table = [0, 0],
      tableName = InnerLoadTable,
      fileName = InputDataFile)
    if not DINcalc annotation (Placement(transformation(extent={{-206,-64},
            {-186,-44}})));

  Modelica.Blocks.Interfaces.RealOutput ZoneTemperatures[NumberZones]  "Temperatures of neighbouring building zones"
    annotation (Placement(transformation(extent={{98,-50},{118,-30}})));
protected
  Real AngleBound[Bound](quantity="Geometry.Angle", displayUnit="°")
    "Radiation angle to boundary"
    annotation(Dialog(group="Others", tab="Results 1", __esi_showAs=ShowAs.Result));
  Real qvHeat(quantity="Thermics.VolumeFlow", displayUnit="m3/h")
    "Volume flow of heating system"
    annotation(Dialog(group="Others", tab="Results 1", __esi_showAs=ShowAs.Result));
  Real CZoneAir(quantity="Thermodynamics.SpecHeatCapacity", displayUnit="kJ/(kg K)")
    "Air heat capacity of the zone"
    annotation(Dialog(group="Others", tab="Results 1", __esi_showAs=ShowAs.Result));
  Real CZoneMass(quantity="Thermodynamics.SpecHeatCapacity", displayUnit="kJ/(kg K)")
    "Mass heat capacity of the zone"
    annotation(Dialog(group="Others", tab="Resuslts 1", __esi_showAs=ShowAs.Result));

protected


  // part 4
public
  IBPSA.ThermalZones.ReducedOrder.RC.FourElements fourElements(
    redeclare package Medium = Modelica.Media.Air.SimpleAir, // 19 Dec 10.00 Modelica.Media.Air.SimpleAir, // 16 Dec 18.00Modelica.Media.Air.MoistAir, // 15 Dec 20.00 IBPSA.Media.Air, // 15 Dec 11.00 Modelica.Media.Air.SimpleAir,
    hConWin=alphaBoundIn1,
    hConExt=alphaBoundIn1,
    hConFloor=2.5, // 21 Dec FIX v11: Added default convective heat transfer coefficient for floor
    hConRoof=2.5,  // 21 Dec FIX v11: Added default convective heat transfer coefficient for roof
    VAir=AZone*hZone,
    hRad=5,
    nOrientations=1,
    AWin=fill(AWindow1, 1),
    ATransparent=fill(AWindow1, 1),
    RWin=1/(uWindow1*(AWindow1 + AWindow2 + AWindow3) + 1E-6),
    gWin=gWindow1,
    ratioWinConRad=0.09,
    AExt=fill(ABound1 - (AWindow1 + AWindow2 + AWindow3), 1),
    nExt=1,
    RExt=fill(dBound1/(uBound1*(ABound1 - (AWindow1 + AWindow2 + AWindow3)) + 1e-6), 1),
    RExtRem=0.1265217391,
    CExt=fill(rhoBound1*cpBound1*dBound1*(ABound1 - (AWindow1 + AWindow2 + AWindow3)), 1),
    AInt=0,
    AFloor=ABound2,
    nFloor=1,
    RFloor=fill(dBound2/(uBound2*ABound2 + 1E-6), 1),
    RFloorRem=0.1265217391,
    CFloor=fill(rhoBound2*cpBound2*dBound2*ABound2, 1),
    ARoof=ABound3,
    nRoof=1,
    RRoof=fill(dBound3/(uBound3*ABound3 + 1E-6), 1),
    RRoofRem=0.1265217391,
    CRoof=fill(rhoBound3*cpBound3*dBound3*ABound3, 1))
    annotation (Placement(transformation(extent={{-20,-18},{28,18}})));

  parameter CoSES_Thermal_ProHMo_PHiL.Interfaces.EnvironmentConditions EnvironmentConditions
  annotation (Placement(
        transformation(extent={{-96,-84},{-74,-62}}), iconTransformation(extent={{-96,-84},
      {-74,-62}})));

  // added for issue with numberPerson
  Modelica.Blocks.Sources.Constant zeroNP(k=0);
  Modelica.Blocks.Sources.Constant zeroBase(k=0);
  Modelica.Blocks.Sources.Constant zeroNorm(k=0);
  Modelica.Blocks.Sources.Constant zeroMachine(k=0);
  Modelica.Blocks.Sources.Constant zeroLight(k=0);
  Modelica.Blocks.Sources.Constant zeroInner(k=0);
  Modelica.Blocks.Sources.Constant zeroSolRad[fourElements.nOrientations](each k=0);


protected
  Real TZoneAct(
    quantity="Thermics.Temp",
    displayUnit="°C",
    start = TZoneInit,
    fixed = true)
    "Actual zone temperature"
    annotation(Dialog(group="Temperature", tab="Results 1", __esi_showAs=ShowAs.Result));

  Real TAmbientBound[Bound](quantity="Thermics.Temp", displayUnit="°C")
    "Ambient temperature per boundary"
    annotation(Dialog(group="Temperature", tab="Results 1", __esi_showAs=ShowAs.Result));
  Real TFlowHeat(quantity="Thermics.Temp", displayUnit="°C")
    "Flow temperature of heating system"
    annotation(Dialog(group="Temperature", tab="Results 1", __esi_showAs=ShowAs.Result));
  Real TReturnHeat(quantity="Thermics.Temp", displayUnit="°C")
    "Return temperature of heating system"
    annotation(Dialog(group="Temperature", tab="Results 1", __esi_showAs=ShowAs.Result));

  Real deltaTBound[Bound](quantity="Thermics.TempDiff", displayUnit="K")
    "Temperature difference across each boundary"
    annotation(Dialog(group="Temperature", tab="Results 1", __esi_showAs=ShowAs.Result));

protected
  Real QHeatCoolLoad(quantity="Basics.Power", displayUnit="kW")
    "Heating/cooling load of building"
    annotation(Dialog(group="Heating and Cooling Power", tab="Results 1", __esi_showAs=ShowAs.Result));


protected
  Real QTotal(quantity="Basics.Power", displayUnit="kW")
    "Total heating/cooling load"
    annotation(Dialog(group="Heating and Cooling Power", tab="Results 1", __esi_showAs=ShowAs.Result));
public
  Real PVent(quantity="Basics.Power", displayUnit="kW")
    "Effective ventilation power"
    annotation(Dialog(group="Electrical Power", tab="Results 1", __esi_showAs=ShowAs.Result));
protected
  Real QVent(quantity="Basics.Power", displayUnit="kW")
    "Reactive ventilation power"
    annotation(Dialog(group="Electrical Power", tab="Results 1", __esi_showAs=ShowAs.Result));
public
  Real Pel(quantity="Basics.Power", displayUnit="kW")
    "Electrical effective power"
    annotation(Dialog(group="Electrical Power", tab="Results 1", __esi_showAs=ShowAs.Result));
protected
  Real Qel(quantity="Basics.Power", displayUnit="kW")
    "Electrical reactive power"
    annotation(Dialog(group="Electrical Power", tab="Results 1", __esi_showAs=ShowAs.Result));
public

  Real Eel(quantity="Basics.Energy", displayUnit="kWh")
    "Electrical energy demand"
    annotation(Dialog(group="Energy", tab="Results 1", __esi_showAs=ShowAs.Result));
protected
  Real QPerson(quantity="Basics.Power", displayUnit="kW")
    "Inner heat yields and losses by persons"
    annotation(Dialog(group="Inner yields and losses - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QLight(quantity="Basics.Power", displayUnit="kW")
    "Inner heat yields by installed light system"
    annotation(Dialog(group="Inner yields and losses - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QMachine(quantity="Basics.Power", displayUnit="kW")
    "Inner heat yields by installed machines"
    annotation(Dialog(group="Inner yields and losses - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QNorm(quantity="Basics.Power", displayUnit="kW")
    "Norm heat yields/losses"
    annotation(Dialog(group="Inner yields and losses - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QInner(quantity="Basics.Power", displayUnit="kW")
    "Inner heat yields/losses"
    annotation(Dialog(group="Inner yields and losses - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QBase(quantity="Basics.Power", displayUnit="kW")
    "Basic heat yields/losses"
    annotation(Dialog(group="Inner yields and losses - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QelHeat(quantity="Basics.Power", displayUnit="kW")
    "Electrical power causing inner heat gains"
    annotation(Dialog(group="Inner yields and losses - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QTransWindowAbsorp[Bound](quantity="Basics.Power", displayUnit="kW")
    "Window transmission losses correction vector for solar yields"
    annotation(Dialog(group="Window - Heating and Cooling", tab="Results 2",
                      __esi_showAs=ShowAs.Result));
  Real QTransWindow[Bound](quantity="Basics.Power", displayUnit="kW")
    "Transmission losses vector for window"
    annotation(Dialog(group="Window - Heating and Cooling", tab="Results 2",
                      __esi_showAs=ShowAs.Result));
  Real QTransOthers[Bound](quantity="Basics.Power", displayUnit="kW")
    "Transmission losses vector for other surfaces"
    annotation(Dialog(group="Walls and other boundaries - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QBound[Bound](quantity="Basics.Power", displayUnit="kW")
    "Heat power of Boundary"
    annotation(Dialog(group="Walls and other boundaries - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));

  Real QBoundAbsorp[Bound](quantity="Basics.Power", displayUnit="kW")
    "Transmission losses correction vector for Boundaries due to radiation absorption"
    annotation(Dialog(group="Walls and other boundaries - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));

  Real QLAirLeak(quantity="Basics.Power", displayUnit="kW")
    "Ventilation losses by air leak"
    annotation(Dialog(group="Ventilation - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QAirComfort(quantity="Basics.Power", displayUnit="kW")
    "Ventilation losses by comfortable air flow"
    annotation(Dialog(group="Ventilation - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QHeatBridge[Bound](quantity="Basics.Power", displayUnit="kW")
    "Transmission losses vector for heat bridge losses"
    annotation(Dialog(group="Heat Bridge - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  Real QHeatBridgeAbsorp[Bound](quantity="Basics.Power", displayUnit="kW")
    "Transmission losses correction vector for heat bridges due to radiation absorption"
    annotation(Dialog(group="Heat Bridge - Heating and Cooling",
                      tab="Results 2", __esi_showAs=ShowAs.Result));
  parameter Boolean Ground1 = if Bound > 0 then (if not contactBound1 > 0 then groundContact1 else false) else false
    "Auxiliary parameter for parameter dialogue initialization ('depthBound1')"
    annotation(Dialog(group="Ground Definitions", tab="Model Initialization"));
  parameter Boolean Ground2 = if Bound > 1 then (if not contactBound2 > 0 then groundContact2 else false) else false
    annotation(Dialog(group="Ground Definitions", tab="Model Initialization"));
  parameter Boolean Ground3 = if Bound > 2 then (if not contactBound3 > 0 then groundContact3 else false) else false
    annotation(Dialog(group="Ground Definitions", tab="Model Initialization"));
public
  parameter Integer Bound = 3 "Number of Boundaries"
    annotation(                                      // 6 to 3
               Dialog(group="Boundaries", tab="Model Initialization"));
protected
  parameter Integer BoundMax=6 "Maximum number of Bounderies"
    annotation(                                               // 9 to 6
               Dialog(
   group="Boundaries",
   tab="Model Initialization"));
  parameter Integer BoundMin=1 "Minimum number of Boundaries"
    annotation(Dialog(
   group="Boundaries",
   tab="Model Initialization"));
public
  parameter Integer Mass = 1 "Number of inner Masses"
    annotation(Dialog(group="Inner Masses", tab="Model Initialization"));
protected
  parameter Integer MassMax=4 "Maximum number of inner Masses"
    annotation(Dialog(group="Inner Masses",tab="Model Initialization"));
  parameter Integer MassMin=0 "Minimum number of inner Masses"
    annotation(Dialog(group="Inner Masses",tab="Model Initialization"));
public
  parameter Real TZoneInit(quantity="Thermics.Temp", displayUnit="°C") = 294.149 if not LoadCalculation
    "Initial zone temperature"
    annotation(Dialog(group="Temperatures", tab="Model Initialization"));
  parameter Real TReturnHeatInit(quantity="Thermics.Temp", displayUnit="°C") = 303.149 if not LoadCalculation and Heat
    "Initial return temperature of heating system"
    annotation(Dialog(group="Temperatures", tab="Model Initialization"));

  parameter Boolean useDayTime = true
    "If enabled, database = HourOfDay, else = HourOfYear"
    annotation(Dialog(group="Database", tab="Model Initialization"));
  parameter Boolean useWindowShading = false
    "External shading of windows is enabled"
    annotation(Dialog(group="Shading", tab="Model Initialization"));
  parameter Real AZone(quantity="Geometry.Area", displayUnit="m²") = 100
    "Net floor space of the zone"
    annotation(Dialog(group="Zone Dimensions", tab="Zone"));
  parameter Real hZone(quantity="Geometry.Length", displayUnit="m") = 2.5
    "Zone height"
    annotation(Dialog(group="Zone Dimensions", tab="Zone"));
  parameter Real PLightInstall(quantity="Basics.Power", displayUnit="W") = 300 if DINcalc
    "Installed electrical power of light"
    annotation(Dialog(group="Installed Electric Devices", tab="Zone"));
  parameter Real PMachineInstall(quantity="Basics.Power", displayUnit="W") = 1000 if DINcalc
    "Installed electrical power of machines"
    annotation(Dialog(group="Installed Electric Devices", tab="Zone"));
  parameter Real etaMachine(quantity="Basics.RelMagnitude", displayUnit="%") = 0.4 if DINcalc
    "Efficiency of machines"
    annotation(Dialog(group="Installed Electric Devices", tab="Zone"));
public
  parameter Boolean Heat = true
    "Heating is enabled/disabled"
    annotation(Dialog(group="Heating Enabling", tab="Heating System"));
  parameter Real cpMedHeat(quantity="Thermics.SpecHeatCapacity", displayUnit="kJ/(kg·K)") = 4177
    if not LoadCalculation and Heat
    "Specific heat capacity of heat medium"
    annotation(Dialog(group="Heating Medium", tab="Heating System"));
  parameter Real rhoMedHeat(quantity="Thermics.Density", displayUnit="kg/m³") = 1000
    if not LoadCalculation and Heat
    "Density of heating medium"
    annotation(Dialog(group="Heating Medium", tab="Heating System"));
  parameter Real n = 1.1
    if not LoadCalculation and Heat
    "Heating system exponent (1.1: floor heating, 1.2-1.3: panel radiator, 1.25: ribbed radiator, 1.3: radiator, 1.25-1.45: convector)"
    annotation(Dialog(group="Heating System", tab="Heating System"));
  parameter Real TFlowHeatNorm(quantity="Thermics.Temp", displayUnit="K") = 308.149
    if not LoadCalculation and Heat
    "Nominal heating flow temperature"
    annotation(                                                                     //45
               Dialog(group="Heating System", tab="Heating System"));
  parameter Real TReturnHeatNorm(quantity="Thermics.Temp", displayUnit="K") = 301.149
    if not LoadCalculation and Heat
    "Nominal heating return temperature"
    annotation(                                                                       //25
               Dialog(group="Heating System", tab="Heating System"));
  parameter Real TZoneNorm(quantity="Thermics.Temp", displayUnit="K") = 293.149
    if not LoadCalculation and Heat
    "Nominal zone temperature"
    annotation(Dialog(group="Heating System", tab="Heating System"));
  parameter Real QHeatNorm(quantity="Thermics.HeatFlowSurf", displayUnit="W/m²") = 50
    if not LoadCalculation and Heat
    "Nominal heat load per floor area"
    annotation(Dialog(group="Heating System Dimensions", tab="Heating System"));
  parameter Real VHeatMedium(quantity="Geometry.Volume", displayUnit="l") = 0.10000000000000001
    if not LoadCalculation and Heat
    "Volume of heating system"
    annotation(Dialog(group="Heating System Dimensions", tab="Heating System"));
  parameter Real QBody(quantity="Basics.Power", displayUnit="W") = 80
    "Body heat dissipation per person"
    annotation(Dialog(group="Inner Yields", tab="Heat Yields and Losses"));
  parameter Real QPersonColdWater(quantity="Basics.Power", displayUnit="W") = -30
    "Individual losses by cold water use"
    annotation(Dialog(group="Inner Yields", tab="Heat Yields and Losses"));
  parameter Real QPersonElectricity(quantity="Basics.Power", displayUnit="W") = 60
    "Electric equipment use per person"
    annotation(Dialog(group="Inner Yields", tab="Heat Yields and Losses"));
  parameter Real LAirLeak(quantity="Basics.Gradient", displayUnit="1/h") = 0.0001389
    "Infiltration air leakage"
    annotation(Dialog(group="Ventilation Losses", tab="Heat Yields and Losses"));
  parameter Real LComfortVentilation(quantity="Thermics.VolumeFlow", displayUnit="m³/h") = 0.0069444444444444441
    "Ventilation air flow"
    annotation(Dialog(group="Ventilation Losses", tab="Heat Yields and Losses"));
  parameter Boolean useVentilationSystem = false
    "If true, use mechanical ventilation"
    annotation(Dialog(group="Ventilation System", tab="Heat Yields and Losses"));
  parameter Real VentilationHeatExchangeRate(quantity="Basics.RelMagnitude", displayUnit="%") = 0.70000000000000007
    "Heat recovery efficiency of ventilation"
    annotation(Dialog(group="Ventilation System", tab="Heat Yields and Losses"));
  parameter Real VentPower(quantity="Basics.Power", displayUnit="W") = 1500
    "Ventilation power"
    annotation( Dialog(group="Ventilation System", tab="Heat Yields and Losses"));
  parameter Real cosPhiVent=0.7
    "Power factor of ventilation system"
    annotation(Dialog(group="Ventilation System",tab="Heat Yields and Losses"));
  parameter Real uHeatBridge(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)") = 0.05
    "Equivalent additional U-value for thermal bridges"
    annotation(Dialog(group="Heat Bridge Losses", tab="Heat Yields and Losses"));
  parameter Real VMass1(quantity="Geometry.Volume", displayUnit="m³") = 6
    "Inner thermal mass volume 1"
    annotation(Dialog(group="Inner Mass 1", tab="Inner Masses"));
  parameter Real cpMass1(quantity="Thermics.SpecHeatCapacity", displayUnit="kJ/(kg·K)") = 920
    "Specific heat capacity of mass 1"
    annotation(Dialog(group="Inner Mass 1", tab="Inner Masses"));
  parameter Real rhoMass1(quantity="Thermics.Density", displayUnit="kg/m³") = 1800
    "Density of inner mass 1"
    annotation(Dialog(group="Inner Mass 1", tab="Inner Masses"));
  parameter Real VMass2(quantity="Geometry.Volume", displayUnit="m³") = 6
    "Inner thermal mass volume 1"
    annotation(Dialog(group="Inner Mass 1", tab="Inner Masses"));
  parameter Real cpMass2(quantity="Thermics.SpecHeatCapacity", displayUnit="kJ/(kg·K)") = 920
    "Specific heat capacity of mass 1"
    annotation(Dialog(group="Inner Mass 1", tab="Inner Masses"));
  parameter Real rhoMass2(quantity="Thermics.Density", displayUnit="kg/m³") = 1800
    "Density of inner mass 1"
    annotation(Dialog(group="Inner Mass 1", tab="Inner Masses"));
  parameter Real infiltrationRate(unit="1/h") = 0.5
    "Air change rate for infiltration";
  parameter Modelica.Units.SI.HeatFlowRate occupancyLoad = 0
    "Heat gains from occupants";
  parameter Modelica.Units.SI.HeatFlowRate applianceLoad = 0
    "Heat gains from appliances";


protected
function ground
  input Real depthGround;
  input Real DayOfYear;
  input Boolean LeapYear;
  input Real cGround;
  input Real lambdaGround;
  input Real rhoGround;
  input Real GeoGradient;
  input Real TAmbientAverage;
  input Real TAmbientMax;
  input Integer MaxMonth;
  output Real TGround;
protected
  Real a = lambdaGround/(rhoGround*cGround);
  Real xi;
  Real TimePeriod;
  Real PhaseDisplacement = 2*pi*MaxMonth/12;
algorithm
  TimePeriod := if LeapYear then 366*24*3600 else 365*24*3600;
  xi := depthGround*sqrt(pi/(a*TimePeriod));
  TGround :=
    TAmbientAverage
    + GeoGradient*depthGround
    + ((TAmbientMax - TAmbientAverage)*exp(-xi))
      * cos(2*pi*(DayOfYear*24*3600)/TimePeriod - PhaseDisplacement - xi);
  annotation(derivative=groundDot, Impure=false);
end ground;

function angleMinMax
  input Real angle;
  input Real angleMax;
  input Real angleMin;
  output Real angleOut;
algorithm
  angleOut := min(angleMax, max(angleMin, angle));
  annotation(Impure=false);
end angleMinMax;

function ThreeVectorMult
  input Real vector1[:];
  input Real vector2[:];
  input Real vector3[:];
  input Integer length;
  output Real prod;
algorithm
  prod := 0;
  for i in 1:length loop
    prod := prod + (vector1[i]*vector2[i]*vector3[i]);
  end for;
  annotation(Impure=false);
end ThreeVectorMult;

function sumVector
  input Real u[:];
  input Integer first;
  input Integer last;
  output Real y;
algorithm
  y := 0;
  for i in first:last loop
    y := y + u[i];
  end for;
  annotation(Impure=false);
end sumVector;

function sumVector1
  input Real u[:];
  input Real v[:];
  input Integer first;
  input Integer last;
  output Real y;
algorithm
  y := 0;
  for i in first:last loop
    y := y + (if v[i] > 1e-3 then u[i] else 0);
  end for;
  annotation(Impure=false);
end sumVector1;

function sumVector2
  input Real u[:];
  input Real v[:];
  input Integer first;
  input Integer last;
  output Real y;
algorithm
  y := 0;
  for i in first:last loop
    y := y + (if not (v[i] > 1e-3) then u[i] else 0);
  end for;
  annotation(Impure=false);
end sumVector2;

function normalVector
  input Real alphaInclination;
  input Real alphaOrientation;
  output Real vector[3];
algorithm
  vector := {
    sin(alphaInclination)*cos(alphaOrientation),
    sin(alphaInclination)*sin(alphaOrientation),
    cos(alphaInclination)};
  annotation(Impure=false);
end normalVector;

function diffAngle
  input Real vector1[3];
  input Real vector2[3];
  output Real angle;
protected
  Real value;
  Real minMax;
algorithm
  value := (vector1[1]*vector2[1] + vector1[2]*vector2[2] + vector1[3]*vector2[3])
           /(sqrt(vector1[1]^2 + vector1[2]^2 + vector1[3]^2)
             *sqrt(vector2[1]^2 + vector2[2]^2 + vector2[3]^2));
  minMax := max(min(value, 1), -1);
  angle := acos(minMax);
  annotation(Impure=false);
end diffAngle;
public

model Boundary "Properties of a boundary V1.0"
  parameter Real ABound(quantity="Geometry.Area", displayUnit="m²")
    "Surface area";
  parameter Real AWindow(quantity="Geometry.Area", displayUnit="m²")
    "Window surface area";
  parameter Real AOthers(quantity="Geometry.Area", displayUnit="m²")
    "Other surfaces (e.g. doors)";
  parameter Real rhoBound(quantity="Thermodynamics.Density", displayUnit="kg/m³")
    "Density of boundary";
  parameter Real cpBound(quantity="Thermodynamics.SpecHeatCapacity", displayUnit="kJ/(kg·K)")
    "Specific heat capacity of boundary";
  parameter Real dBound(quantity="Geometry.Length", displayUnit="m")
    "Thickness of boundary";
  parameter Real gWindow "Total energy translucency of window";
  parameter Real uBound(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)")
    "Heat transmission value of boundary";
  parameter Real uWindow(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)")
    "Heat transmission value of window";
  parameter Real uOthers(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)")
    "Heat transmission value of other surfaces";
  parameter Real epsDirt "Dirt correction value for window";
  parameter Real epsShading "Shading correction value for window";
  parameter Real epsFrame "Frame correction value for window";
  parameter Real NormalVector[3] "Normal vector of boundary";
  parameter Real alphaInc "Inclination angle of boundary";
  parameter Real alphaOr  "Orientation angle of boundary";
  parameter Real alphaBound "Absorption coefficient of boundary";
  parameter Real alphaBoundOut(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)")
    "Outer heat transmission coefficient of boundary";
  parameter Real alphaBoundIn(quantity="Thermics.HeatTransmCoeff", displayUnit="W/(m²·K)")
    "Inner heat transmission coefficient of boundary";
  parameter Real depthBound(quantity="Geometry.Length", displayUnit="m")
    "Depth of boundary, if it is ground connected";
  parameter Integer contactBound
    "Boundary is connected to: '0' - ambience, 1,2,3 ... - Zone 1,2,3";
  /* // 17 Dec 11.00   // 17 Dec 10.00 added 5 statements ------------------------------------------------------------------
  // Exposed boundary results (filled by parent model HeatedZone_1)
  // These are NOT parameters and NOT outputs: parent assigns them by equations.
  // ------------------------------------------------------------------
  Real TBoundIn(
    quantity="Thermics.Temp",
    displayUnit="°C",
    start=293.15)
    "Inner surface temperature of boundary"
    annotation(Dialog(group="Temperature", tab="Results 1", __esi_showAs=ShowAs.Result));

  Real TBoundOut(
    quantity="Thermics.Temp",
    displayUnit="°C",
    start=283.15)
    "Outer surface temperature of boundary"
    annotation(Dialog(group="Temperature", tab="Results 1", __esi_showAs=ShowAs.Result));

  Real QBoundIn(
    quantity="Basics.Power",
    displayUnit="kW")
    "Inner heat losses/gains due to boundary (W internally)"
    annotation(Dialog(group="Walls and other boundaries - Heating and Cooling",
      tab="Results 2", __esi_showAs=ShowAs.Result));

  Real QBoundOut(
    quantity="Basics.Power",
    displayUnit="kW")
    "Outer heat losses/gains due to boundary (W internally)"
    annotation(Dialog(group="Walls and other boundaries - Heating and Cooling",
      tab="Results 2", __esi_showAs=ShowAs.Result));

  Real QBoundChange(
    quantity="Basics.Power",
    displayUnit="kW")
    "Heat exchanged through boundary (W internally)"
    annotation(Dialog(group="Walls and other boundaries - Heating and Cooling",
      tab="Results 2", __esi_showAs=ShowAs.Result));
*/
    /* 16 Dec 10.00
    // 15 Dec 14.00 removed input
    // 14 Dec 22.00 - ACTIVE TBoundIn/TBoundOut declarations
    Real TBoundIn(
      quantity="Thermics.Temp",
      displayUnit="°C",
      start=293.15)
      "Inner surface temperature of boundary";

    Real TBoundOut(
      quantity="Thermics.Temp",
      displayUnit="°C",
      start=283.15)
      "Outer surface temperature of boundary";
*/
    /*    
    // 13 Dec 21.00 added
    Real TBoundIn(
      quantity="Thermics.Temp",
      displayUnit="°C")
      "Average inner boundary temperature (set by parent model)";

    Real TBoundOut(
      quantity="Thermics.Temp",
      displayUnit="°C")
      "Average outer boundary temperature (set by parent model)";
    */
    /* 14 Dec 21.00 commented
     // 13 Dec 17.00 commented
    // 13 Dec 16.00 added
    Real TBoundIn(
      quantity="Thermics.Temp",
      displayUnit="°C",
      start=293.15)
      "Inner surface temperature of boundary";

    Real TBoundOut(
      quantity="Thermics.Temp",
      displayUnit="°C",
      start=283.15)
      "Outer surface temperature of boundary";
*/
    /* // 13 Dec 12.00
    // 10 Dec 17.00
    Real TBoundIn(
      quantity="Thermics.Temp",
      displayUnit="°C")
      "Average inner boundary temperature (placeholder, not used in equations)";

    Real TBoundOut(
      quantity="Thermics.Temp",
      displayUnit="°C")
      "Average outer boundary temperature (placeholder, not used in equations)";
  */
  // 17 Dec 14.00
  parameter Modelica.Units.SI.Temperature TGround = 283.15
    "Ground temperature if boundary is ground-connected";
  /* // 17 Dec 14.00
  Real TGround( // 17 Dec 10.00 parameter to Real TGround
    quantity="Thermics.Temp",
    displayUnit="°C",
    start=283.15)
  "Ground temperature, if boundary is ground connected (placeholder)";
  */
    /* // 10 Dec 12.00
    Real TBoundIn(quantity="Thermics.Temp", displayUnit="°C")
      "Average inner boundary temperature";
    Real TBoundOut(quantity="Thermics.Temp", displayUnit="°C")
      "Average outer boundary temperature";
    Real TGround(quantity="Thermics.Temp", displayUnit="°C")
      "Ground temperature, if boundary is ground connected";
*/
  parameter Real lambdaBound(quantity="Thermics.SpecHeatCond", displayUnit="W/(m·K)")
    "Heat conductance of boundary";
  parameter Boolean groundContact
    "Boundary ground contact enabled/disabled";

  annotation(Icon(coordinateSystem(extent={{-100,50},{100,-50}})));
end Boundary;



  // part 6
public
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow internalGains
    annotation (Placement(transformation(extent={{-84,-6},{-64,16}})));

  Modelica.Blocks.Sources.RealExpression TInSig[Bound](each y=TZoneAct)
    annotation (Placement(transformation(extent={{-42,70},{-22,90}})));

  Modelica.Blocks.Sources.RealExpression ToutSig(y = 273.15 + T_outside.y)
    annotation (Placement(transformation(extent={{-98,-52},{-78,-32}})));

  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature TOutSrc
    "Outdoor temperature for infiltration"
    annotation (Placement(transformation(extent={{-54,-48},{-34,-28}})));



  // ******************heatPort exists, but is NOT connected internally***********************
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
    "Thermal port to connect hydronic system to the zone"
    annotation (Placement(transformation(extent={{-116,32},{-96,52}})));


  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor senTZone
    annotation (Placement(transformation(extent={{-140,-32},{-120,-12}})));

  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow QIntRad
    annotation (Placement(transformation(extent={{-50,14},{-30,34}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor HVAC
    annotation (Placement(transformation(extent={{152,14},{172,34}})));
  // ============================================================================
  // VARIABLE INTERNAL GAINS INPUTS (28 Dec - CORRECTED)
  // ============================================================================
  Modelica.Blocks.Interfaces.RealInput nPersons(start=0)
    "Number of persons in zone"
    annotation (Placement(transformation(
        extent={{-14,-14},{14,14}},
        rotation=180,
        origin={166,60})));

  Modelica.Blocks.Interfaces.RealInput P_appliances_W(start=50)
    "Appliance power [W]"
    annotation (Placement(transformation(extent={{-168,42},{-140,70}}),
        iconTransformation(extent={{-168,42},{-140,70}})));

  Modelica.Blocks.Interfaces.RealInput Q_radiation_W(start=0)
    "Internal radiative heat gain [W]"
    annotation (Placement(transformation(extent={{-170,-70},{-142,-42}}),
        iconTransformation(extent={{-170,-70},{-142,-42}})));

  Modelica.Blocks.Math.Gain occGain_conv(k=80)
    "Convert number of persons to heat [W] (80W per person)"
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=180,
        origin={129,59})));

initial equation

  // Boundary 1 checks
  assert(uBound1 > 1e-10, "Boundary 1 - uBound = 0");
  assert(alphaBoundIn1 > 1e-10, "Boundary 1 - alphaBoundIn = 0");
  assert(contactBound1 >= 0 and contactBound1 <= NumberZones,
         "Boundary 1 - contactBound out of bounds");
  assert(alphaInclination1 >= 0 and alphaInclination1 <= pi,
         "Boundary 1 - alphaInclination out of range");
  assert(alphaOrientation1 >= 0 and alphaOrientation1 <= 2*pi,
         "Boundary 1 - alphaOrientation out of range");

  // Boundary 2 checks
  assert(uBound2 > 1e-10, "Boundary 2 - uBound = 0");
  assert(alphaBoundIn2 > 1e-10, "Boundary 2 - alphaBoundIn = 0");
  assert(contactBound2 >= 0 and contactBound2 <= NumberZones,
         "Boundary 2 - contactBound out of bounds");
  assert(alphaInclination2 >= 0 and alphaInclination2 <= pi,
         "Boundary 2 - alphaInclination out of range");
  assert(alphaOrientation2 >= 0 and alphaOrientation2 <= 2*pi,
         "Boundary 2 - alphaOrientation out of range");

  // Boundary 3 checks
  assert(uBound3 > 1e-10, "Boundary 3 - uBound = 0");
  assert(alphaBoundIn3 > 1e-10, "Boundary 3 - alphaBoundIn = 0");
  assert(contactBound3 >= 0 and contactBound3 <= NumberZones,
         "Boundary 3 - contactBound out of bounds");
  assert(alphaInclination3 >= 0 and alphaInclination3 <= pi,
         "Boundary 3 - alphaInclination out of range");
  assert(alphaOrientation3 >= 0 and alphaOrientation3 <= 2*pi,
         "Boundary 3 - alphaOrientation out of range");

  assert(Bound >= BoundMin, "Number of Boundaries is too low");
  assert(Bound <= BoundMax, "Number of Boundaries is too high");
  assert(Mass  >= MassMin,  "Number of inner Masses is too low");
  assert(Mass  <= MassMax,  "Number of inner Masses is too high");
  assert(NumberZones >= 1 and NumberZones <= MaxNumberZones, "Number of Zones out of limits");
  assert(ZoneIndex   >= 1 and ZoneIndex   <= MaxNumberZones, "Zone index out of limits");



//algorithm
equation
  HeatCoolLoad = -HVAC.Q_flow;
  // Set to 0 because HVAC already provides heating/cooling through its separate connection
  internalGains.Q_flow = 0;

  // Basic electrical outputs (no electrical model)
  Pel = 0;
  QelHeat = 0;
  Qel = 0;

  // Add equations for ALL missing result variables
  Eel = 0;
  PVent = 0;
  QAirComfort = 0;
  QBase = 0;
  QHeatCoolLoad = -HVAC.Q_flow;
QInner = 0;
  QLAirLeak = 0;
  QLight = 0;
  QMachine = 0;
  QNorm = 0;
  QPerson = 0;
  QTotal = 0;
  QVent = 0;
  qvHeat = 0;
  TFlowHeat = 0;
  TReturnHeat = 0;
  TOutSrc.T = ToutSig.y;  // Still needed since connect(ToutSig.y, TOutSrc.T) is commented out


  // Add equation for TZoneAct - it was never assigned!
  TZoneAct = TZone;

  // Map fourElements air temperature to external output
  TZone = fourElements.TAir;
  ZoneTemperatures = fill(TZone, NumberZones);

  // ---------------------------------------------------------
  // 1. Compute deltaTBound[i]
  // ---------------------------------------------------------
  for i in 1:Bound loop
    if (Boundaries[i].contactBound > 0 or Boundaries[i].groundContact) then
      if (Boundaries[i].lambdaBound > 1e-10) then
        deltaTBound[i] = (TZoneAct - TAmbientBound[i])
                         * Boundaries[i].uBound / (Boundaries[i].lambdaBound / Boundaries[i].dBound);
      else
        deltaTBound[i] = TZoneAct - TAmbientBound[i];
      end if;
    else
      if (Boundaries[i].lambdaBound > 1e-10) then
        deltaTBound[i] = (TZoneAct - EnvironmentConditions.TAmbientAverageAct)
                         * Boundaries[i].uBound / (Boundaries[i].lambdaBound / Boundaries[i].dBound);
      else
        deltaTBound[i] = TZoneAct - EnvironmentConditions.TAmbientAverageAct;
      end if;
    end if;
  end for;

for i in 1:Bound loop
  if Boundaries[i].groundContact then
    TGround[i] = EnvironmentConditions.TAverageAmbientAnnual;
    TAmbientBound[i] = TGround[i];
  else
    TGround[i] = EnvironmentConditions.TAmbient;
    TAmbientBound[i] = EnvironmentConditions.TAmbient;
  end if;
end for;

  CZoneAir  =  EnvironmentConditions.cpAir * EnvironmentConditions.rhoAir * AZone * hZone;
  if Mass > 0 then
    CZoneMass =  ThreeVectorMult(Masses.cp, Masses.rho, Masses.V, Mass);
  else
    CZoneMass =  0;
  end if;


/* --------------------------------------------------------------
   OLD SIMX HEAT-TRANSFER LOOP — DISABLED FOR PHIL VERSION
--------------------------------------------------------------  
*/
for i in (1:Bound) loop
  AngleBound[i] = angleMinMax(
    diffAngle(EnvironmentConditions.RadiationVector, Boundaries[i].NormalVector),
    1.5707963267948966, 0);

  if useWindowShading then
    QTransWindowAbsorp[i] =
      -Boundaries[i].gWindow * (1 - WindowShading[i]) * Boundaries[i].AWindow *
      (1 - Boundaries[i].epsDirt) * (1 - Boundaries[i].epsShading) * (1 - Boundaries[i].epsFrame) *
      ((EnvironmentConditions.RadiationDirect + 0.25*EnvironmentConditions.RadiationDiffuse) * cos(AngleBound[i])
        + 0.75*EnvironmentConditions.RadiationDiffuse * 0.5*(cos(Boundaries[i].alphaInc)+1));
  else
    QTransWindowAbsorp[i] =
      -Boundaries[i].gWindow * Boundaries[i].AWindow *
      (1 - Boundaries[i].epsDirt) * (1 - Boundaries[i].epsShading) * (1 - Boundaries[i].epsFrame) *
      ((EnvironmentConditions.RadiationDirect + 0.25*EnvironmentConditions.RadiationDiffuse) * cos(AngleBound[i])
        + 0.75*EnvironmentConditions.RadiationDiffuse * 0.5*(cos(Boundaries[i].alphaInc)+1));
  end if;

  QTransWindow[i] =
    (Boundaries[i].uWindow + uHeatBridge) * Boundaries[i].AWindow * (TZoneAct - TAmbientBound[i]);

  QTransOthers[i] =
    (Boundaries[i].uOthers + uHeatBridge) * Boundaries[i].AOthers * (TZoneAct - TAmbientBound[i]);

  if Boundaries[i].groundContact then
    QHeatBridgeAbsorp[i] =  0;
    QBoundAbsorp[i] =  0;
  else
    QHeatBridgeAbsorp[i] =
      -uHeatBridge * (1 - Boundaries[i].epsShading) *
      (Boundaries[i].ABound - Boundaries[i].AWindow - Boundaries[i].AOthers) *
      (Boundaries[i].alphaBound / Boundaries[i].alphaBoundOut) *
      ((EnvironmentConditions.RadiationDirect + 0.25*EnvironmentConditions.RadiationDiffuse) * cos(AngleBound[i])
        + 0.75*EnvironmentConditions.RadiationDiffuse * 0.5*(cos(Boundaries[i].alphaInc)+1));

    QBoundAbsorp[i] =
      -Boundaries[i].uBound * (1 - Boundaries[i].epsShading) *
      (Boundaries[i].ABound - Boundaries[i].AWindow - Boundaries[i].AOthers) *
      (Boundaries[i].alphaBound / Boundaries[i].alphaBoundOut) *
      ((EnvironmentConditions.RadiationDirect + 0.25*EnvironmentConditions.RadiationDiffuse) * cos(AngleBound[i])
        + 0.75*EnvironmentConditions.RadiationDiffuse * 0.5*(cos(Boundaries[i].alphaInc)+1));
  end if;

  //--- Inner boundary heat flow ---
  QBoundIn[i] =
    -Boundaries[i].alphaBoundIn *
    (Boundaries[i].ABound - Boundaries[i].AWindow - Boundaries[i].AOthers) *
    (TZoneAct - TAmbientBound[i]);

  // --- Heat transfer through boundary (ALWAYS defined) ---
  if Boundaries[i].dBound > 0.001 then
    QBoundChange[i] =
      -Boundaries[i].lambdaBound / Boundaries[i].dBound *
      (Boundaries[i].ABound - Boundaries[i].AWindow - Boundaries[i].AOthers) *
      (TAmbientBound[i] - TZoneAct);
  else
    QBoundChange[i] = 0;
  end if;

  // --- Outer boundary heat flow
  if Boundaries[i].groundContact then
    QBoundOut[i] = QBoundChange[i];
  else
    QBoundOut[i] =
      Boundaries[i].alphaBoundOut *
      (Boundaries[i].ABound - Boundaries[i].AWindow - Boundaries[i].AOthers) *
      (TAmbientBound[i] - TZoneAct)
      + QBoundAbsorp[i];
  end if;


  QBound[i] =
    Boundaries[i].uBound *
    (Boundaries[i].ABound - Boundaries[i].AWindow - Boundaries[i].AOthers) *
    (TZoneAct - TAmbientBound[i])
    + QBoundAbsorp[i];

  // Added QHeatBridge calculation
  QHeatBridge[i] =
    uHeatBridge *
    (Boundaries[i].ABound - Boundaries[i].AWindow - Boundaries[i].AOthers) *
    (TZoneAct - TAmbientBound[i]);

end for;

// --- DYNAMIC OR MASSLESS BOUNDARY TEMPERATURES ---
for i in 1:Bound loop
  if Boundaries[i].dBound > 1e-3 then
    // Dynamic internal node
    (Boundaries[i].ABound
     - Boundaries[i].AWindow
     - Boundaries[i].AOthers)
    * Boundaries[i].dBound/2
    * Boundaries[i].rhoBound
    * Boundaries[i].cpBound
    * der(TBoundIn[i])
      = QBoundIn[i] - QBoundChange[i];

    if Boundaries[i].groundContact then
      der(TBoundOut[i]) = der(TAmbientBound[i]);
    else
      (Boundaries[i].ABound
       - Boundaries[i].AWindow
       - Boundaries[i].AOthers)
      * Boundaries[i].dBound/2
      * Boundaries[i].rhoBound
      * Boundaries[i].cpBound
      * der(TBoundOut[i])
        = -QBoundOut[i] + QBoundChange[i];
    end if;

  else
    // Massless boundary
    TBoundIn[i]  = TZoneAct;
    TBoundOut[i] = TAmbientBound[i];
  end if;
end for;

// ============================================================================
// HeatedZone.mo - COMPLETE ACTIVE CONNECT STATEMENTS SECTION
// ============================================================================

  // -------------------------------------------------------------------------
  // HEAT PORT CONNECTIONS (Thermal)
  // -------------------------------------------------------------------------
  // 1. Appliance load connected to internal convective gains
  connect(fourElements.intGainsConv, appLoad.port) annotation (Line(points={{28,4},{
          34,4},{34,64},{10,64}},      color={191,0,0}));

  // 2. Occupant load connected to internal convective gains
  connect(fourElements.intGainsConv, occLoad.port) annotation (Line(points={{28,4},{
          34,4},{34,18},{56,18},{56,76},{80,76},{80,66}},     color={191,0,0}));

  // 3. Internal gains heat port connected to convective gains
  connect(internalGains.port, fourElements.intGainsConv) annotation (Line(
        points={{-64,5},{-26,5},{-26,24},{34,24},{34,4},{28,4}}, color={191,0,0}));

  // 4. Infiltration loss port_a connected to internal convective gains
  connect(infiltrationLoss.port_a, fourElements.intGainsConv) annotation (Line(
        points={{54,3},{42,3},{42,4},{28,4}},     color={191,0,0}));

  // 5. External temperature source connected to infiltration loss port_b

  // -------------------------------------------------------------------------
  // SIGNAL CONNECTIONS (Real values)
  // -------------------------------------------------------------------------

  // 7. Zero constants connected to load inputs (no annotations - simple connections)
  connect(zeroNP.y, NumberPerson.u[1]);
  connect(zeroBase.y, BaseLoad.u[1]);
  connect(zeroNorm.y, NormLoad.u[1]);
  connect(zeroMachine.y, MachineLoad.u[1]);
  connect(zeroLight.y, LightLoad.u[1]);
  connect(zeroInner.y, InnerLoad.u[1]);

  // 8. Zero solar radiation connected to fourElements
  connect(zeroSolRad.y, fourElements.solRad);


  // 10. Occupant load Q_flow from constant

  // 11. Appliance load Q_flow from constant

  // 12. Infiltration air temperature from outside temperature signal
  connect(T_infiltration_air.T, ToutSig.y) annotation (Line(points={{90,0},{90,-58},
          {-78,-58},{-78,-42},{-77,-42}}, color={0,0,127}));



  connect(senTZone.port, fourElements.intGainsConv) annotation (Line(points={{-140,
          -22},{-146,-22},{-146,-4},{-106,-4},{-106,-12},{-26,-12},{-26,24},{34,
          24},{34,4},{28,4}}, color={191,0,0}));

  connect(T_infiltration_air.port, infiltrationLoss.port_b) annotation (Line(
        points={{112,0},{118,0},{118,16},{80,16},{80,3},{72,3}},     color={191,
          0,0}));


  connect(QIntRad.port, fourElements.intGainsRad) annotation (Line(points={{-30,
          24},{-24,24},{-24,26},{36,26},{36,8},{28,8}}, color={191,0,0}));

  connect(HVAC.port_b, fourElements.intGainsConv) annotation (Line(points={{172,
          24},{172,84},{30,84},{30,44},{34,44},{34,4},{28,4}}, color={191,0,0}));


  // ============================================================================
  // VARIABLE INTERNAL GAINS CONNECTIONS (28 Dec - CORRECTED)
  // ============================================================================
  connect(nPersons, occGain_conv.u);
  connect(occGain_conv.y, occLoad.Q_flow);
  connect(P_appliances_W, appLoad.Q_flow);
  connect(Q_radiation_W, QIntRad.Q_flow);

  connect(heatPort, HVAC.port_a) annotation (Line(points={{-106,42},{78,42},{78,
          20},{146,20},{146,24},{152,24}}, color={191,0,0}));
end HeatedZone;
