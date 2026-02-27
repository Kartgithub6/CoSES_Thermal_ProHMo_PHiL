within CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis;
model ThreeZoneBuilding_optimizedNEW
  "Three-zone building with Optimized hydraulic topology - reorganized: Roof → Living → Cellar"

  inner Modelica.Fluid.System system(
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    T_ambient=278.15)
    annotation(Placement(visible=true, transformation(origin={-170,170}, extent={{-10,-10},{10,10}}, rotation=0)));
  replaceable package Medium = Modelica.Media.Water.StandardWater;

  // ============================================================================
  // FLUID PORTS
  // ============================================================================
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    "Supply water inlet"
    annotation(Placement(visible=true, transformation(origin={-200,60},extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-100,60}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    "Return water outlet"
    annotation(Placement(visible=true, transformation(origin={-200,-62},  extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-100,-60}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // OUTPUTS - System Performance
  // ============================================================================
  Modelica.Blocks.Interfaces.RealOutput qvRef
    "Total volume flow rate [m³/s]"
    annotation(Placement(visible=true, transformation(origin={210,-100}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-70}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ⭐ REORGANIZED ORDER: Roof → Living → Cellar
  Modelica.Blocks.Interfaces.RealOutput TZone_roof
    "Roof office temperature [K]"
    annotation(Placement(visible=true, transformation(origin={210,-60}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,0}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput TZone_living
    "Living room temperature [K]"
    annotation(Placement(visible=true, transformation(origin={210,40}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,30}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput TZone_cellar
    "Cellar temperature [K]"
    annotation(Placement(visible=true, transformation(origin={210,140}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,60}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_roof_opening
    "Roof valve position [0-1]"
    annotation(Placement(visible=true, transformation(origin={210,-140}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-90}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_living_opening
    "Living valve position [0-1]"
    annotation(Placement(visible=true, transformation(origin={210,0}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-50}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_cellar_opening
    "Cellar valve position [0-1]"
    annotation(Placement(visible=true, transformation(origin={210,100}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-30}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // INPUTS - Occupancy (Roof → Living → Cellar)
  // ============================================================================
  Modelica.Blocks.Interfaces.RealInput nPersons_roof(start=0)
    "Number of people in roof office"
    annotation(Placement(visible=true, transformation(origin={-220,118}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,20}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput nPersons_living(start=0)
    "Number of people in living room"
    annotation(Placement(visible=true, transformation(origin={-220,140}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,50}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput nPersons_cellar(start=0)
    "Number of people in cellar"
    annotation(Placement(visible=true, transformation(origin={-220,180}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,80}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // INPUTS - Appliance Loads (Roof → Living → Cellar)
  // ============================================================================
  Modelica.Blocks.Interfaces.RealInput P_appliances_roof_W(start=50)
    "Appliance power in roof office [W]"
    annotation(Placement(visible=true, transformation(origin={230,100}, extent={{-10,-10},{10,10}}, rotation=180),
      iconTransformation(origin={110,20}, extent={{-10,-10},{10,10}}, rotation=180)));

  Modelica.Blocks.Interfaces.RealInput P_appliances_living_W(start=200)
    "Appliance power in living room [W]"
    annotation(Placement(visible=true, transformation(origin={230,140}, extent={{-10,-10},{10,10}}, rotation=180),
      iconTransformation(origin={110,50}, extent={{-10,-10},{10,10}}, rotation=180)));

  Modelica.Blocks.Interfaces.RealInput P_appliances_cellar_W(start=50)
    "Appliance power in cellar [W]"
    annotation(Placement(visible=true, transformation(origin={230,180}, extent={{-10,-10},{10,10}}, rotation=180),
      iconTransformation(origin={110,80}, extent={{-10,-10},{10,10}}, rotation=180)));

  // ============================================================================
  // INPUTS - Solar Radiation per Zone (from IBPSA weather/solar chain)
  // ============================================================================
  Modelica.Blocks.Interfaces.RealInput solRad_roof[1]
    "Solar radiation through windows for roof zone [W/m2]"
    annotation(Placement(visible=true, transformation(origin={-220,-60}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,-20}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput solRad_living[1]
    "Solar radiation through windows for living zone [W/m2]"
    annotation(Placement(visible=true, transformation(origin={-220,-80}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,-50}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput solRad_cellar[1]
    "Solar radiation through windows for cellar zone [W/m2]"
    annotation(Placement(visible=true, transformation(origin={-220,-100}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,-80}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // PARAMETERS - Zone Geometry (Roof → Living → Cellar)
  // ============================================================================
  parameter Modelica.Units.SI.Area AZone_roof=60 "Roof office floor area";
  parameter Modelica.Units.SI.Area AZone_living=100 "Living room floor area";
  parameter Modelica.Units.SI.Area AZone_cellar=80 "Cellar floor area";
  parameter Modelica.Units.SI.Length hZone_roof=2.3 "Roof office height";
  parameter Modelica.Units.SI.Length hZone_living=2.5 "Living room height";
  parameter Modelica.Units.SI.Length hZone_cellar=2.2 "Cellar height";

  // ============================================================================
  // PARAMETERS - Temperature Setpoints (Roof → Living → Cellar)
  // ============================================================================
  parameter Modelica.Units.SI.Temperature TRef_roof=293.15 "Roof setpoint 20°C";
  parameter Modelica.Units.SI.Temperature TRef_living=293.15 "Living setpoint 20°C";
  parameter Modelica.Units.SI.Temperature TRef_cellar=291.15 "Cellar setpoint 18°C";
  parameter Modelica.Units.SI.Temperature TZoneInit_roof=290.15 "Roof initial temperature";
  parameter Modelica.Units.SI.Temperature TZoneInit_living=293.15 "Living initial temperature";
  parameter Modelica.Units.SI.Temperature TZoneInit_cellar=291.15 "Cellar initial temperature";

  // ============================================================================
  // PARAMETERS - Control
  // ============================================================================
  parameter Boolean roofHeat=true "Enable roof heating";
  parameter Boolean cellarHeat=true "Enable cellar heating";
  parameter Real k_PI=0.3 "PI proportional gain";
  parameter Modelica.Units.SI.Time Ti_PI=500 "PI integral time";

  // OPTIMIZED VALVE MINIMUMS (Roof → Living → Cellar)
  parameter Real yMin_roof=0.02 " 18 Jan Reduced to 3% Roof valve minimum 8%";
  parameter Real yMin_living=0.02 "Living valve minimum 3% (priority zone)";
  parameter Real yMin_cellar=0.01 "Reduced from 5% to 2% Cellar valve minimum 5% (REDUCED from 20%!)";

  // ============================================================================
  // FLUID SENSORS
  // ============================================================================
  Modelica.Fluid.Sensors.VolumeFlowRate qvMedium(redeclare package Medium=Medium)
    "Volumetric flow rate sensor"
    annotation(Placement(visible=true, transformation(origin={-168,60},extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Sensors.TemperatureTwoPort TMedium(redeclare package Medium=Medium)
    "Supply temperature sensor"
    annotation(Placement(visible=true, transformation(origin={-136,60},extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Sensors.TemperatureTwoPort TReturn(redeclare package Medium=Medium)
    "Return temperature sensor"
    annotation(Placement(visible=true, transformation(origin={160,-100}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // HYDRAULIC TOPOLOGY - OPTIMIZED DESIGN
  // ============================================================================
  // Main tee: Splits flow between LIVING (priority) and CELLAR+ROOF (shared)
  Modelica.Fluid.Fittings.TeeJunctionIdeal teeMain(redeclare package Medium=Medium)
    "Main flow splitter: Living vs Cellar+Roof"
    annotation(Placement(visible=true, transformation(origin={-110,100}, extent={{-10,-10},{10,10}}, rotation=0)));

  // Secondary tee: Splits remaining flow between cellar and roof
  Modelica.Fluid.Fittings.TeeJunctionIdeal teeCellarRoof(redeclare package Medium=Medium)
    "Secondary splitter: Cellar vs Roof"
    annotation(Placement(visible=true, transformation(origin={-70,75}, extent={{-10,-10},{10,10}}, rotation=0)));

  // Merge tee: Combines cellar and roof returns
  Modelica.Fluid.Fittings.TeeJunctionIdeal teeMergeCellarRoof(redeclare package Medium=Medium)
    "Merge cellar and roof returns"
    annotation(Placement(visible=true, transformation(origin={90,75}, extent={{-10,10},{10,-10}}, rotation=0)));

  // Final merge: Combines all returns
  Modelica.Fluid.Fittings.TeeJunctionIdeal teeMergeMain(redeclare package Medium=Medium)
    "Merge all zone returns"
    annotation(Placement(visible=true, transformation(origin={130,-100}, extent={{-10,10},{10,-10}}, rotation=0)));

  // ============================================================================
  // CONTROL VALVES (Roof → Living → Cellar)
  // ============================================================================
  Modelica.Fluid.Valves.ValveLinear valve_roof(
    redeclare package Medium=Medium,
    dp_nominal=10000,
    m_flow_nominal=0.04,
    m_flow(start=0.008),
    m_flow_small=0.001)
    "Roof office control valve"
    annotation(Placement(visible=true, transformation(origin={-50,-5}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Valves.ValveLinear valve_living(
    redeclare package Medium=Medium,
    dp_nominal=10000,
    m_flow_nominal=0.06,
    m_flow(start=0.02),
    m_flow_small=0.001)
    "Living room control valve (PRIORITY)"
    annotation(Placement(visible=true, transformation(origin={-50,50}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Valves.ValveLinear valve_cellar(
    redeclare package Medium=Medium,
    dp_nominal=10000,
    m_flow_nominal=0.04,
    m_flow(start=0.008),
    m_flow_small=0.001)
    "Cellar control valve"
    annotation(Placement(visible=true, transformation(origin={-50,125}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // PI CONTROLLERS (Roof → Living → Cellar)
  // ============================================================================
  Modelica.Blocks.Continuous.LimPID PI_roof(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=k_PI,
    Ti=Ti_PI,
    yMax=if roofHeat then 1.0 else 0.0,
    yMin=if roofHeat then yMin_roof else 0.0)
    "Roof office temperature controller"
    annotation(Placement(visible=true, transformation(origin={-90,25}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Continuous.LimPID PI_living(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=k_PI,
    Ti=Ti_PI,
    yMax=1.0,
    yMin=yMin_living)
    "Living room temperature controller"
    annotation(Placement(visible=true, transformation(origin={-96,72}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Continuous.LimPID PI_cellar(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=k_PI,
    Ti=Ti_PI,
    yMax=if cellarHeat then 1.0 else 0.0,
    yMin=if cellarHeat then yMin_cellar else 0.0)
    "Cellar temperature controller"
    annotation(Placement(visible=true, transformation(origin={-90,150}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // HYDRONIC SUBSYSTEMS with RIGHT-SIZED radiators (Roof → Living → Cellar)
  // ============================================================================
  CoSES_Thermal_ProHMo_PHiL.HydronicSystem.SystemWithZoneAndHydronics_optimized roof_hydSys(
    redeclare package Medium=Medium,
    Q_flow_nominal=2000,  // Roof: 2500W (adequate)
    T_a_nominal=343.15,
    T_b_nominal=323.15,
    TAir_nominal=293.15,
    fraRad=0.35,
    nEle=5,
    T_start=323.15,
    p_start=200000)
    "Roof office hydronic system"
    annotation(Placement(visible=true, transformation(origin={-2,11},extent={{-20,-20},{20,20}}, rotation=0)));

  CoSES_Thermal_ProHMo_PHiL.HydronicSystem.SystemWithZoneAndHydronics_optimized living_hydSys(
    redeclare package Medium=Medium,
    Q_flow_nominal=3500,  // Living: 3500W (priority zone)
    T_a_nominal=343.15,
    T_b_nominal=323.15,
    TAir_nominal=293.15,
    fraRad=0.35,
    nEle=5,
    T_start=323.15,
    p_start=200000)
    "Living room hydronic system"
    annotation(Placement(visible=true, transformation(origin={0,38},extent={{-20,-20},{20,20}}, rotation=0)));

  CoSES_Thermal_ProHMo_PHiL.HydronicSystem.SystemWithZoneAndHydronics_optimized cellar_hydSys(
    redeclare package Medium=Medium,
    Q_flow_nominal=1500,  // Reduced from 2500 W Cellar: 2500W (REDUCED from 4000W!)
    T_a_nominal=343.15,
    T_b_nominal=323.15,
    TAir_nominal=291.15,
    fraRad=0.35,
    nEle=5,
    T_start=323.15,
    p_start=200000)
    "Cellar hydronic system"
    annotation(Placement(visible=true, transformation(origin={0,135}, extent={{-20,-20},{20,20}}, rotation=0)));

  // ============================================================================
  // BUILDING ZONES - Using BuildingSystem library (Roof → Living → Cellar)
  // ============================================================================
  BuildingSystem.Building.HeatedZone roof_zone(
    ZoneIndex=3,
    NumberZones=3,
    AZone=AZone_roof,
    hZone=hZone_roof,
    TZoneInit=TZoneInit_roof,
    infiltrationRate=0.20,  // Low-moderate infiltration
    occupancyLoad=100,
    applianceLoad=50)
    "Roof office thermal zone"
    annotation(Placement(visible=true, transformation(origin={72,-59},extent={{-20,-20},{20,20}}, rotation=0)));

  BuildingSystem.Building.HeatedZone living_zone(
    ZoneIndex=2,
    NumberZones=3,
    AZone=AZone_living,
    hZone=hZone_living,
    TZoneInit=TZoneInit_living,
    infiltrationRate=0.30,  // Moderate infiltration
    occupancyLoad=200,
    applianceLoad=150)
    "Living room thermal zone"
    annotation(Placement(visible=true, transformation(origin={86,78}, extent={{-20,-20},{20,20}}, rotation=0)));

  BuildingSystem.Building.HeatedZone cellar_zone(
    ZoneIndex=1,
    NumberZones=3,
    AZone=AZone_cellar,
    hZone=hZone_cellar,
    TZoneInit=TZoneInit_cellar,
    infiltrationRate=0.15,  // Low infiltration (underground)
    occupancyLoad=50,
    applianceLoad=50)
    "Cellar thermal zone"
    annotation(Placement(visible=true, transformation(origin={70,135}, extent={{-20,-20},{20,20}}, rotation=0)));

  // ============================================================================
  // SETPOINT CONSTANTS (Roof → Living → Cellar)
  // ============================================================================
  Modelica.Blocks.Sources.Constant TRefConst_roof(k=TRef_roof)
    "Roof temperature setpoint"
    annotation(Placement(visible=true, transformation(origin={-152,-29},  extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Sources.Constant TRefConst_living(k=TRef_living)
    "Living temperature setpoint"
    annotation(Placement(visible=true, transformation(origin={-158,8},extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Sources.Constant TRefConst_cellar(k=TRef_cellar)
    "Cellar temperature setpoint"
    annotation(Placement(visible=true, transformation(origin={-160,150}, extent={{-10,-10},{10,10}}, rotation=0)));

  // Window shading (disabled for now) - Roof → Living → Cellar
  Modelica.Blocks.Sources.Constant WindowShadingConst_roof[3](each k=0)
    annotation(Placement(visible=true, transformation(origin={50,25}, extent={{-10,-10},{10,10}}, rotation=0)));
  Modelica.Blocks.Sources.Constant WindowShadingConst_living[3](each k=0)
    annotation(Placement(visible=true, transformation(origin={50,80}, extent={{-10,-10},{10,10}}, rotation=0)));
  Modelica.Blocks.Sources.Constant WindowShadingConst_cellar[3](each k=0)
    annotation(Placement(visible=true, transformation(origin={50,150}, extent={{-10,-10},{10,10}}, rotation=0)));

  // Solar radiation (disabled for now) - Roof → Living → Cellar
  Modelica.Blocks.Sources.Constant Q_radiation_roof(k=0)
    annotation(Placement(visible=true, transformation(origin={30,35}, extent={{-10,-10},{10,10}}, rotation=0)));
  Modelica.Blocks.Sources.Constant Q_radiation_living(k=0)
    annotation(Placement(visible=true, transformation(origin={30,90}, extent={{-10,-10},{10,10}}, rotation=0)));
  Modelica.Blocks.Sources.Constant Q_radiation_cellar(k=0)
    annotation(Placement(visible=true, transformation(origin={30,160}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // FLOW RATE APPROXIMATION (Roof → Living → Cellar)
  // ============================================================================
  Modelica.Blocks.Math.Add3 qvSum
    "Sum of all zone flow rates"
    annotation(Placement(visible=true, transformation(origin={170,-100}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Math.Gain qv_roof_approx(k=0.001)
    "Roof flow approximation"
    annotation(Placement(visible=true, transformation(origin={130,25}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Math.Gain qv_living_approx(k=0.001)
    "Living flow approximation"
    annotation(Placement(visible=true, transformation(origin={130,80}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Math.Gain qv_cellar_approx(k=0.001)
    "Cellar flow approximation"
    annotation(Placement(visible=true, transformation(origin={136,150}, extent={{-10,-10},{10,10}}, rotation=0)));

equation
  // ============================================================================
  // HYDRAULIC CONNECTIONS - OPTIMIZED TOPOLOGY
  // ============================================================================

  // Supply path: Port_a → Sensors → Main tee
  connect(port_a, qvMedium.port_a)
    annotation(Line(points={{-200,60},{-178,60}}, color={0,127,255}, thickness=0.5));
  connect(qvMedium.port_b, TMedium.port_a)
    annotation(Line(points={{-158,60},{-146,60}}, color={0,127,255}, thickness=0.5));
  connect(TMedium.port_b, teeMain.port_1)
    annotation(Line(points={{-126,60},{-118,60},{-118,84},{-126,84},{-126,100},{-120,100}}, color={0,127,255}, thickness=0.5));

  // LIVING PRIORITY: Direct connection from main tee
  connect(teeMain.port_2, valve_living.port_a)
    annotation(Line(points={{-100,100},{-60,100},{-60,50}}, color={0,127,255}, thickness=0.5));
  connect(valve_living.port_b, living_hydSys.port_a)
    annotation(Line(points={{-40,50},{-30,50},{-30,38},{-20,38}}, color={0,127,255}, thickness=0.5));

  // CELLAR+ROOF SHARED: Secondary branch from main tee
  connect(teeMain.port_3, teeCellarRoof.port_1)
    annotation(Line(points={{-110,110},{-110,114},{-86,114},{-86,75},{-80,75}}, color={0,127,255}));
  connect(teeCellarRoof.port_2, valve_cellar.port_a)
    annotation(Line(points={{-60,75},{-60,125},{-60,125}}, color={0,127,255}));
  connect(teeCellarRoof.port_3, valve_roof.port_a)
    annotation(Line(points={{-70,85},{-70,-5},{-60,-5}}, color={0,127,255}));
  connect(valve_cellar.port_b, cellar_hydSys.port_a)
    annotation(Line(points={{-40,125},{-30,125},{-30,135},{-20,135}}, color={0,127,255}));
  connect(valve_roof.port_b, roof_hydSys.port_a)
    annotation(Line(points={{-40,-5},{-30,-5},{-30,11},{-22,11}}, color={0,127,255}));

  // Return path merging
  connect(cellar_hydSys.port_b, teeMergeCellarRoof.port_2)
    annotation(Line(points={{20,135},{50,135},{50,75},{100,75}},color={0,127,255}));
  connect(roof_hydSys.port_b, teeMergeCellarRoof.port_3)
    annotation(Line(points={{18,11},{50,11},{50,65},{90,65}}, color={0,127,255}));
  connect(teeMergeCellarRoof.port_1, teeMergeMain.port_3)
    annotation(Line(points={{80,75},{130,75},{130,-110}}, color={0,127,255}));
  connect(living_hydSys.port_b, teeMergeMain.port_2)
    annotation(Line(points={{20,38},{40,38},{40,-118},{140,-118},{140,-100}}, color={0,127,255}));
  connect(teeMergeMain.port_1, TReturn.port_a)
    annotation(Line(points={{120,-100},{150,-100}}, color={0,127,255}, thickness=0.5));
  connect(TReturn.port_b, port_b)
    annotation(Line(points={{170,-100},{170,-120},{-200,-120},{-200,-62}}, color={0,127,255}, thickness=0.5));

  // ============================================================================
  // ZONE THERMAL CONNECTIONS (Roof → Living → Cellar)
  // ============================================================================
  connect(roof_hydSys.port_zone, roof_zone.heatPort)
    annotation(Line(points={{-2,31},{-2,34},{-72,34},{-72,-46},{42,-46},{42,-50.6},{50.8,-50.6}}, color={191,0,0}));
  connect(living_hydSys.port_zone, living_zone.heatPort)
    annotation(Line(points={{0,58},{0,66},{34,66},{34,86.4},{64.8,86.4}}, color={191,0,0}));
  connect(cellar_hydSys.port_zone, cellar_zone.heatPort)
    annotation(Line(points={{0,154},{0,160},{50,160},{50,143.4},{48.8,143.4}}, color={191,0,0}));

  // ============================================================================
  // CONTROL CONNECTIONS - PI Controllers (Roof → Living → Cellar)
  // ============================================================================

  // Roof control loop
  connect(TRefConst_roof.y, PI_roof.u_s)
    annotation(Line(points={{-141,-29},{-114,-29},{-114,25},{-102,25}}, color={0,0,127}));
  connect(roof_zone.TZone, PI_roof.u_m)
    annotation(Line(points={{60,-79},{60,-88},{-90,-88},{-90,13}}, color={0,0,127}));
  connect(PI_roof.y, valve_roof.opening)
    annotation(Line(points={{-79,25},{-70,25},{-70,3},{-50,3}}, color={0,0,127}));
  connect(TRefConst_roof.y, roof_zone.TZoneRef)
    annotation(Line(points={{-141,-29},{-120,-29},{-120,-94},{100,-94},{100,-51},{91.6,-51}}, color={0,0,127}));

  // Living control loop
  connect(TRefConst_living.y, PI_living.u_s)
    annotation(Line(points={{-147,8},{-118,8},{-118,72},{-108,72}}, color={0,0,127}));
  connect(living_zone.TZone, PI_living.u_m)
    annotation(Line(points={{74,58},{74,50},{-96,50},{-96,60}}, color={0,0,127}));
  connect(PI_living.y, valve_living.opening)
    annotation(Line(points={{-85,72},{-70,72},{-70,58},{-50,58}}, color={0,0,127}));
  connect(TRefConst_living.y, living_zone.TZoneRef)
    annotation(Line(points={{-147,8},{-118,8},{-118,58},{-110,58},{-110,84},
          {-88,84},{-88,92},{46,92},{46,98},{105.6,98},{105.6,86}}, color={0,0,127}));

  // Cellar control loop
  connect(TRefConst_cellar.y, PI_cellar.u_s)
    annotation(Line(points={{-149,150},{-102,150}}, color={0,0,127}));
  connect(cellar_zone.TZone, PI_cellar.u_m)
    annotation(Line(points={{58,115},{110,115},{110,130},{-90,130},{-90,138}}, color={0,0,127}));
  connect(PI_cellar.y, valve_cellar.opening)
    annotation(Line(points={{-79,150},{-70,150},{-70,140},{-50,140},{-50,133}}, color={0,0,127}));
  connect(TRefConst_cellar.y, cellar_zone.TZoneRef)
    annotation(Line(points={{-149,150},{-140,150},{-140,180},{100,180},{100,143},{89.6,143}}, color={0,0,127}));

  // ============================================================================
  // ZONE INPUT CONNECTIONS (Roof → Living → Cellar)
  // ============================================================================

  // Window shading
  connect(WindowShadingConst_roof.y, roof_zone.WindowShading)
    annotation(Line(points={{61,25},{66,25},{66,-78.8},{51.4,-78.8}}, color={0,0,127}));
  connect(WindowShadingConst_living.y, living_zone.WindowShading)
    annotation(Line(points={{61,80},{65.4,80},{65.4,58.2}}, color={0,0,127}));
  connect(WindowShadingConst_cellar.y, cellar_zone.WindowShading)
    annotation(Line(points={{61,150},{49.4,150},{49.4,115.2}}, color={0,0,127}));

  // Solar radiation
  connect(Q_radiation_roof.y, roof_zone.Q_radiation_W)
    annotation(Line(points={{41,35},{41,-69.4},{49.6,-69.4}}, color={0,0,127}));
  connect(Q_radiation_living.y, living_zone.Q_radiation_W)
    annotation(Line(points={{41,90},{63.6,90},{63.6,67.6}}, color={0,0,127}));
  connect(Q_radiation_cellar.y, cellar_zone.Q_radiation_W)
    annotation(Line(points={{41,160},{50,160},{50,124.6},{47.6,124.6}},
                                                                  color={0,0,127}));

  // Occupancy
  connect(nPersons_roof, roof_zone.nPersons)
    annotation(Line(points={{-220,118},{25,118},{25,-46.6},{96,-46.6}},color={0,0,127}));
  connect(nPersons_living, living_zone.nPersons)
    annotation(Line(points={{-220,140},{-210,140},{-210,130},{30,130},{30,90.4},
          {110,90.4}}, color={0,0,127}));
  connect(nPersons_cellar, cellar_zone.nPersons)
    annotation(Line(points={{-220,180},{-200,180},{-200,190},{40,190},{40,147.4},
          {94,147.4}}, color={0,0,127}));

  // Appliance loads
  connect(P_appliances_roof_W, roof_zone.P_appliances_W)
    annotation(Line(points={{230,100},{194,100},{194,-46.2},{49.2,-46.2}}, color={0,0,127}));
  connect(P_appliances_living_W, living_zone.P_appliances_W)
    annotation(Line(points={{230,140},{146,140},{146,104},{63.2,104},{63.2,90.8}}, color={0,0,127}));
  connect(P_appliances_cellar_W, cellar_zone.P_appliances_W)
    annotation(Line(points={{230,180},{220,180},{220,147.8},{47.2,147.8}}, color={0,0,127}));

  // ============================================================================
  // SOLAR RADIATION CONNECTIONS (from IBPSA weather/solar chain)
  // ============================================================================
  connect(solRad_roof, roof_zone.solRadIn)
    annotation(Line(points={{-220,-60},{-190,-60},{-190,-106},{72,-106},{72,-81}},
      color={0,0,127}));
  connect(solRad_living, living_zone.solRadIn)
    annotation(Line(points={{-220,-80},{-194,-80},{-194,-108},{86,-108},{86,56}},
      color={0,0,127}));
  connect(solRad_cellar, cellar_zone.solRadIn)
    annotation(Line(points={{-220,-100},{-198,-100},{-198,-116},{70,-116},{70,113}},
      color={0,0,127}));

  // ============================================================================
  // OUTPUT CONNECTIONS (Roof → Living → Cellar)
  // ============================================================================
  connect(roof_zone.TZone, TZone_roof)
    annotation(Line(points={{60,-79},{60,-88},{114,-88},{114,-84},{210,-84},
          {210,-60}}, color={0,0,127}));
  connect(living_zone.TZone, TZone_living)
    annotation(Line(points={{74,58},{74,50},{190,50},{190,40},{210,40}}, color={0,0,127}));
  connect(cellar_zone.TZone, TZone_cellar)
    annotation(Line(points={{58,115},{210,115},{210,140}}, color={0,0,127}));

  connect(PI_roof.y, valve_roof_opening)
    annotation(Line(points={{-79,25},{-70,25},{-70,70},{180,70},{180,-140},{210,-140}}, color={0,0,127}));
  connect(PI_living.y, valve_living_opening)
    annotation(Line(points={{-85,72},{-70,72},{-70,-44},{228,-44},{228,0},{210,0}}, color={0,0,127}));
  connect(PI_cellar.y, valve_cellar_opening)
    annotation(Line(points={{-79,150},{-70,150},{-70,170},{190,170},{190,100},{210,100}}, color={0,0,127}));

  // Flow rate approximation
  connect(PI_roof.y, qv_roof_approx.u)
    annotation(Line(points={{-79,25},{118,25}}, color={0,0,127}));
  connect(PI_living.y, qv_living_approx.u)
    annotation(Line(points={{-85,72},{20,72},{20,80},{118,80}}, color={0,0,127}));
  connect(PI_cellar.y, qv_cellar_approx.u)
    annotation(Line(points={{-79,150},{124,150}}, color={0,0,127}));

  connect(qv_roof_approx.y, qvSum.u3)
    annotation(Line(points={{141,25},{150,25},{150,-108},{158,-108}}, color={0,0,127}));
  connect(qv_living_approx.y, qvSum.u2)
    annotation(Line(points={{141,80},{155,80},{155,-100},{158,-100}}, color={0,0,127}));
  connect(qv_cellar_approx.y, qvSum.u1)
    annotation(Line(points={{147,150},{150,150},{150,-92},{158,-92}}, color={0,0,127}));

  connect(qvSum.y, qvRef)
    annotation(Line(points={{181,-100},{210,-100}}, color={0,0,127}));

annotation(
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-70,60},{0,85},{70,60},{70,40},{-70,40},{-70,60}},
          lineColor={0,0,255},
          fillColor={200,220,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Rectangle(
          extent={{-70,40},{70,0}},
          lineColor={0,128,0},
          fillColor={220,255,220},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Rectangle(
          extent={{-70,0},{70,-40}},
          lineColor={139,69,19},
          fillColor={255,235,205},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Line(
          points={{-90,-40},{90,-40}},
          color={100,100,100},
          thickness=2),
        Text(
          extent={{-60,72},{60,62}},
          textString="ROOF",
          textColor={0,0,200},
          textStyle={TextStyle.Bold},
          fontSize=10),
        Text(
          extent={{-60,55},{60,45}},
          textString="Office/Attic",
          textColor={0,0,150},
          fontSize=7),
        Text(
          extent={{-60,32},{60,22}},
          textString="LIVING",
          textColor={0,128,0},
          textStyle={TextStyle.Bold},
          fontSize=10),
        Text(
          extent={{-60,15},{60,5}},
          textString="Main Floor",
          textColor={0,100,0},
          fontSize=7),
        Text(
          extent={{-60,-8},{60,-18}},
          textString="CELLAR",
          textColor={139,69,19},
          textStyle={TextStyle.Bold},
          fontSize=10),
        Text(
          extent={{-60,-25},{60,-35}},
          textString="Basement",
          textColor={100,50,0},
          fontSize=7),
        Line(
          points={{-90,20},{-70,20}},
          color={255,0,0},
          thickness=2,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{70,20},{90,20}},
          color={0,0,255},
          thickness=2,
          arrow={Arrow.None,Arrow.Filled}),
        Text(
          extent={{-90,100},{90,85}},
          textString="%name",
          textColor={0,0,0},
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-90,-50},{90,-65}},
          textString="Living Priority",
          textColor={0,128,0},
          textStyle={TextStyle.Bold},
          fontSize=9),
        Text(
          extent={{-90,-70},{90,-85}},
          textString="Roof → Living → Cellar",
          textColor={100,100,100},
          fontSize=7),
        Rectangle(
          extent={{-50,30},{-40,25}},
          lineColor={135,206,235},
          fillColor={200,230,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-25,30},{-15,25}},
          lineColor={135,206,235},
          fillColor={200,230,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{15,30},{25,25}},
          lineColor={135,206,235},
          fillColor={200,230,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{40,30},{50,25}},
          lineColor={135,206,235},
          fillColor={200,230,255},
          fillPattern=FillPattern.Solid)}),

    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-220,-220},{250,220}})),

    Documentation(info="<html>
<h4>Three-Zone Building Model</h4>

<p><b>Building Structure (Top to Bottom):</b></p>
<ul>
<li><b>ROOF Zone</b> - Attic/upper floor office space</li>
<li><b>LIVING Zone</b> - Main floor living area (PRIORITY ZONE)</li>
<li><b>CELLAR Zone</b> - Basement/storage area</li>
</ul>

<p><b>Hydraulic Priority:</b></p>
<p>Living zone receives heating priority through series hydraulic configuration</p>

<p><b>Flow Path:</b></p>
<p>Supply → Roof → Living → Cellar → Return</p>

<p><b>Control:</b></p>
<p>Independent PI controllers for each zone with configurable setpoints and minimum valve positions (yMin)</p>

<p><b>Parameters from Building Type Record:</b></p>
<ul>
<li>Zone areas and heights</li>
<li>Temperature setpoints and initial conditions</li>
<li>PI controller parameters (k_PI, Ti_PI)</li>
<li>Minimum valve openings (yMin_roof, yMin_living, yMin_cellar)</li>
<li>Default occupancy and appliance loads</li>
</ul>
</html>"));
end ThreeZoneBuilding_optimizedNEW;
