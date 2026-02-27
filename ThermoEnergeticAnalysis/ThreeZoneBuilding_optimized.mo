within CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis;
model ThreeZoneBuilding_optimized
  "Three-zone building with Optimized hydraulic topology - reorganized: Roof -> Living -> Cellar"

  // ============================================================================
  // COORDINATE LAYOUT GUIDE (diagram extent -230,-215 to 260,215)
  //
  //  Zone rows (Y centerlines):   CELLAR = +140 | LIVING = 0 | ROOF = -140
  //
  //  Column X positions:
  //   -222 : external inputs/outputs (nPersons, solRad, port_a/b)
  //   -185 : qvMedium sensor
  //   -155 : TMedium sensor
  //   -175 : TRefConst setpoints  (per-zone row)
  //   -130 : PI controllers       (per-zone row)
  //    -50 : control valves       (per-zone row)
  //      0 : hydSys blocks        (per-zone row)
  //     47 : WindowShading / Q_radiation constants (per-zone row)
  //    110 : building zone blocks (per-zone row)
  //    162 : qv_approx gains      (per-zone row)
  //    158 : teeMergeCellarRoof
  //    175 : teeMergeMain
  //    196 : TReturn
  //    240 : real outputs
  //   -105 : teeMain (supply splitter, Y=70 midway between Living/Cellar)
  //    -80 : teeCellarRoof (Y=70, splits Cellar up / Roof down)
  // ============================================================================

  inner Modelica.Fluid.System system(
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    T_ambient=278.15)
    annotation(Placement(visible=true, transformation(origin={-200,205}, extent={{-10,-10},{10,10}}, rotation=0)));

  replaceable package Medium = Modelica.Media.Water.StandardWater;

  // ============================================================================
  // FLUID PORTS
  // ============================================================================
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    "Supply water inlet"
    annotation(Placement(visible=true, transformation(origin={-222,10}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-100,60}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    "Return water outlet"
    annotation(Placement(visible=true, transformation(origin={-222,-195}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-100,-60}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // OUTPUTS
  // ============================================================================
  Modelica.Blocks.Interfaces.RealOutput TZone_cellar
    "Cellar temperature [K]"
    annotation(Placement(visible=true, transformation(origin={240,140}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,60}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_cellar_opening
    "Cellar valve position [0-1]"
    annotation(Placement(visible=true, transformation(origin={240,115}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-30}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput TZone_living
    "Living room temperature [K]"
    annotation(Placement(visible=true, transformation(origin={240,0}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,30}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_living_opening
    "Living valve position [0-1]"
    annotation(Placement(visible=true, transformation(origin={240,-25}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-50}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput TZone_roof
    "Roof office temperature [K]"
    annotation(Placement(visible=true, transformation(origin={240,-140}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,0}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_roof_opening
    "Roof valve position [0-1]"
    annotation(Placement(visible=true, transformation(origin={240,-165}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-90}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput qvRef
    "Total volume flow rate [m3/s]"
    annotation(Placement(visible=true, transformation(origin={240,-200}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={110,-70}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // INPUTS - Occupancy
  // ============================================================================
  Modelica.Blocks.Interfaces.RealInput nPersons_cellar(start=0)
    "Number of people in cellar"
    annotation(Placement(visible=true, transformation(origin={-222,185}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,80}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput nPersons_living(start=0)
    "Number of people in living room"
    annotation(Placement(visible=true, transformation(origin={-222,165}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,50}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput nPersons_roof(start=0)
    "Number of people in roof office"
    annotation(Placement(visible=true, transformation(origin={-222,145}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,20}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // INPUTS - Appliance Loads
  // ============================================================================
  Modelica.Blocks.Interfaces.RealInput P_appliances_cellar_W(start=50)
    "Appliance power in cellar [W]"
    annotation(Placement(visible=true, transformation(origin={250,140}, extent={{-10,-10},{10,10}}, rotation=180),
      iconTransformation(origin={110,80}, extent={{-10,-10},{10,10}}, rotation=180)));

  Modelica.Blocks.Interfaces.RealInput P_appliances_living_W(start=200)
    "Appliance power in living room [W]"
    annotation(Placement(visible=true, transformation(origin={250,0}, extent={{-10,-10},{10,10}}, rotation=180),
      iconTransformation(origin={110,50}, extent={{-10,-10},{10,10}}, rotation=180)));

  Modelica.Blocks.Interfaces.RealInput P_appliances_roof_W(start=50)
    "Appliance power in roof office [W]"
    annotation(Placement(visible=true, transformation(origin={250,-140}, extent={{-10,-10},{10,10}}, rotation=180),
      iconTransformation(origin={110,20}, extent={{-10,-10},{10,10}}, rotation=180)));

  // ============================================================================
  // INPUTS - Solar Radiation
  // ============================================================================
  Modelica.Blocks.Interfaces.RealInput solRad_roof[1]
    "Solar radiation - roof zone [W/m2]"
    annotation(Placement(visible=true, transformation(origin={-222,-145}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,-20}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput solRad_living[1]
    "Solar radiation - living zone [W/m2]"
    annotation(Placement(visible=true, transformation(origin={-222,-165}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,-50}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput solRad_cellar[1]
    "Solar radiation - cellar zone [W/m2]"
    annotation(Placement(visible=true, transformation(origin={-222,-185}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-110,-80}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // PARAMETERS - Zone Geometry
  // ============================================================================
  parameter Modelica.Units.SI.Area AZone_roof=60;
  parameter Modelica.Units.SI.Area AZone_living=100;
  parameter Modelica.Units.SI.Area AZone_cellar=80;
  parameter Modelica.Units.SI.Length hZone_roof=2.3;
  parameter Modelica.Units.SI.Length hZone_living=2.5;
  parameter Modelica.Units.SI.Length hZone_cellar=2.2;

  // ============================================================================
  // PARAMETERS - Temperature Setpoints
  // ============================================================================
  parameter Modelica.Units.SI.Temperature TRef_roof=293.15;
  parameter Modelica.Units.SI.Temperature TRef_living=293.15;
  parameter Modelica.Units.SI.Temperature TRef_cellar=291.15;
  parameter Modelica.Units.SI.Temperature TZoneInit_roof=290.15;
  parameter Modelica.Units.SI.Temperature TZoneInit_living=293.15;
  parameter Modelica.Units.SI.Temperature TZoneInit_cellar=291.15;

  // ============================================================================
  // PARAMETERS - Control
  // ============================================================================
  parameter Boolean roofHeat=true;
  parameter Boolean cellarHeat=true;
  parameter Real k_PI=0.3;
  parameter Modelica.Units.SI.Time Ti_PI=500;
  parameter Real yMin_roof=0.02;
  parameter Real yMin_living=0.02;
  parameter Real yMin_cellar=0.01;

  // ============================================================================
  // FLUID SENSORS  (supply header, x = -185 and -155, y = 10)
  // ============================================================================
  Modelica.Fluid.Sensors.VolumeFlowRate qvMedium(redeclare package Medium=Medium)
    annotation(Placement(visible=true, transformation(origin={-185,10}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Sensors.TemperatureTwoPort TMedium(redeclare package Medium=Medium)
    annotation(Placement(visible=true, transformation(origin={-155,10}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Sensors.TemperatureTwoPort TReturn(redeclare package Medium=Medium)
    annotation(Placement(visible=true, transformation(origin={196,-195}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // HYDRAULIC SPLITTERS  (x=-105 and x=-80, y=70 between Cellar/Living)
  // ============================================================================
  Modelica.Fluid.Fittings.TeeJunctionIdeal teeMain(redeclare package Medium=Medium)
    "Main splitter: Living vs (Cellar+Roof)"
    annotation(Placement(visible=true, transformation(origin={-105,70}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Fittings.TeeJunctionIdeal teeCellarRoof(redeclare package Medium=Medium)
    "Secondary splitter: Cellar vs Roof"
    annotation(Placement(visible=true, transformation(origin={-80,70}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // HYDRAULIC MERGERS  (x=158 and x=175, y=-70 between Living/Roof)
  // ============================================================================
  Modelica.Fluid.Fittings.TeeJunctionIdeal teeMergeCellarRoof(redeclare package Medium=Medium)
    "Merge Cellar + Roof returns"
    annotation(Placement(visible=true, transformation(origin={158,-70}, extent={{10,-10},{-10,10}}, rotation=0)));

  Modelica.Fluid.Fittings.TeeJunctionIdeal teeMergeMain(redeclare package Medium=Medium)
    "Merge all zone returns"
    annotation(Placement(visible=true, transformation(origin={175,-70}, extent={{10,-10},{-10,10}}, rotation=0)));

  // ============================================================================
  // CONTROL VALVES  (x=-50, per-zone Y)
  // ============================================================================
  Modelica.Fluid.Valves.ValveLinear valve_cellar(
    redeclare package Medium=Medium,
    dp_nominal=10000, m_flow_nominal=0.04, m_flow(start=0.008), m_flow_small=0.001)
    annotation(Placement(visible=true, transformation(origin={-50,140}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Valves.ValveLinear valve_living(
    redeclare package Medium=Medium,
    dp_nominal=10000, m_flow_nominal=0.06, m_flow(start=0.02), m_flow_small=0.001)
    annotation(Placement(visible=true, transformation(origin={-50,0}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Fluid.Valves.ValveLinear valve_roof(
    redeclare package Medium=Medium,
    dp_nominal=10000, m_flow_nominal=0.04, m_flow(start=0.008), m_flow_small=0.001)
    annotation(Placement(visible=true, transformation(origin={-50,-140}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // PI CONTROLLERS  (x=-130, per-zone Y)
  // ============================================================================
  Modelica.Blocks.Continuous.LimPID PI_cellar(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=k_PI, Ti=Ti_PI,
    yMax=if cellarHeat then 1.0 else 0.0,
    yMin=if cellarHeat then yMin_cellar else 0.0)
    annotation(Placement(visible=true, transformation(origin={-130,140}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Continuous.LimPID PI_living(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=k_PI, Ti=Ti_PI,
    yMax=1.0, yMin=yMin_living)
    annotation(Placement(visible=true, transformation(origin={-130,0}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Continuous.LimPID PI_roof(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=k_PI, Ti=Ti_PI,
    yMax=if roofHeat then 1.0 else 0.0,
    yMin=if roofHeat then yMin_roof else 0.0)
    annotation(Placement(visible=true, transformation(origin={-130,-140}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // SETPOINT CONSTANTS  (x=-175, per-zone Y)
  // ============================================================================
  Modelica.Blocks.Sources.Constant TRefConst_cellar(k=TRef_cellar)
    annotation(Placement(visible=true, transformation(origin={-175,140}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Sources.Constant TRefConst_living(k=TRef_living)
    annotation(Placement(visible=true, transformation(origin={-177,-28},
                                                                       extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Sources.Constant TRefConst_roof(k=TRef_roof)
    annotation(Placement(visible=true, transformation(origin={-173,-140}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ============================================================================
  // HYDRONIC SUBSYSTEMS  (x=0, per-zone Y, extent ±20)
  // ============================================================================
  CoSES_Thermal_ProHMo_PHiL.HydronicSystem.SystemWithZoneAndHydronics_optimized cellar_hydSys(
    redeclare package Medium=Medium,
    Q_flow_nominal=1500, T_a_nominal=343.15, T_b_nominal=323.15,
    TAir_nominal=291.15, fraRad=0.35, nEle=5, T_start=323.15, p_start=200000)
    annotation(Placement(visible=true, transformation(origin={0,140}, extent={{-20,-20},{20,20}}, rotation=0)));

  CoSES_Thermal_ProHMo_PHiL.HydronicSystem.SystemWithZoneAndHydronics_optimized living_hydSys(
    redeclare package Medium=Medium,
    Q_flow_nominal=3500, T_a_nominal=343.15, T_b_nominal=323.15,
    TAir_nominal=293.15, fraRad=0.35, nEle=5, T_start=323.15, p_start=200000)
    annotation(Placement(visible=true, transformation(origin={0,0}, extent={{-20,-20},{20,20}}, rotation=0)));

  CoSES_Thermal_ProHMo_PHiL.HydronicSystem.SystemWithZoneAndHydronics_optimized roof_hydSys(
    redeclare package Medium=Medium,
    Q_flow_nominal=2000, T_a_nominal=343.15, T_b_nominal=323.15,
    TAir_nominal=293.15, fraRad=0.35, nEle=5, T_start=323.15, p_start=200000)
    annotation(Placement(visible=true, transformation(origin={0,-140}, extent={{-20,-20},{20,20}}, rotation=0)));

  // ============================================================================
  // BUILDING ZONES  (x=110, per-zone Y, extent ±20)
  // ============================================================================
  BuildingSystem.Building.HeatedZone cellar_zone(
    ZoneIndex=1, NumberZones=3, AZone=AZone_cellar, hZone=hZone_cellar,
    TZoneInit=TZoneInit_cellar, infiltrationRate=0.15, occupancyLoad=50, applianceLoad=50)
    annotation(Placement(visible=true, transformation(origin={110,140}, extent={{-20,-20},{20,20}}, rotation=0)));

  BuildingSystem.Building.HeatedZone living_zone(
    ZoneIndex=2, NumberZones=3, AZone=AZone_living, hZone=hZone_living,
    TZoneInit=TZoneInit_living, infiltrationRate=0.30, occupancyLoad=200, applianceLoad=150)
    annotation(Placement(visible=true, transformation(origin={110,0}, extent={{-20,-20},{20,20}}, rotation=0)));

  BuildingSystem.Building.HeatedZone roof_zone(
    ZoneIndex=3, NumberZones=3, AZone=AZone_roof, hZone=hZone_roof,
    TZoneInit=TZoneInit_roof, infiltrationRate=0.20, occupancyLoad=100, applianceLoad=50)
    annotation(Placement(visible=true, transformation(origin={110,-140}, extent={{-20,-20},{20,20}}, rotation=0)));

  // ============================================================================
  // WINDOW SHADING & SOLAR RADIATION CONSTANTS  (x=47, per-zone Y ±14)
  // ============================================================================
  Modelica.Blocks.Sources.Constant WindowShadingConst_cellar[3](each k=0)
    annotation(Placement(visible=true, transformation(origin={47,154}, extent={{-8,-8},{8,8}}, rotation=0)));
  Modelica.Blocks.Sources.Constant Q_radiation_cellar(k=0)
    annotation(Placement(visible=true, transformation(origin={47,126}, extent={{-8,-8},{8,8}}, rotation=0)));

  Modelica.Blocks.Sources.Constant WindowShadingConst_living[3](each k=0)
    annotation(Placement(visible=true, transformation(origin={47,14}, extent={{-8,-8},{8,8}}, rotation=0)));
  Modelica.Blocks.Sources.Constant Q_radiation_living(k=0)
    annotation(Placement(visible=true, transformation(origin={47,-14}, extent={{-8,-8},{8,8}}, rotation=0)));

  Modelica.Blocks.Sources.Constant WindowShadingConst_roof[3](each k=0)
    annotation(Placement(visible=true, transformation(origin={47,-126}, extent={{-8,-8},{8,8}}, rotation=0)));
  Modelica.Blocks.Sources.Constant Q_radiation_roof(k=0)
    annotation(Placement(visible=true, transformation(origin={47,-154}, extent={{-8,-8},{8,8}}, rotation=0)));

  // ============================================================================
  // FLOW RATE APPROXIMATION  (x=162, per-zone Y)
  // ============================================================================
  Modelica.Blocks.Math.Gain qv_cellar_approx(k=0.001)
    annotation(Placement(visible=true, transformation(origin={162,140}, extent={{-8,-8},{8,8}}, rotation=0)));

  Modelica.Blocks.Math.Gain qv_living_approx(k=0.001)
    annotation(Placement(visible=true, transformation(origin={162,0}, extent={{-8,-8},{8,8}}, rotation=0)));

  Modelica.Blocks.Math.Gain qv_roof_approx(k=0.001)
    annotation(Placement(visible=true, transformation(origin={162,-140}, extent={{-8,-8},{8,8}}, rotation=0)));

  Modelica.Blocks.Math.Add3 qvSum
    annotation(Placement(visible=true, transformation(origin={205,-70}, extent={{-10,-10},{10,10}}, rotation=0)));

equation
  // ============================================================================
  // HYDRAULIC SUPPLY PATH
  // port_a -> qvMedium -> TMedium -> teeMain
  // ============================================================================
  connect(port_a, qvMedium.port_a)
    annotation(Line(points={{-222,10},{-195,10}}, color={0,127,255}, thickness=0.5));
  connect(qvMedium.port_b, TMedium.port_a)
    annotation(Line(points={{-175,10},{-165,10}}, color={0,127,255}, thickness=0.5));
  connect(TMedium.port_b, teeMain.port_1)
    annotation(Line(points={{-145,10},{-130,10},{-130,70},{-115,70}}, color={0,127,255}, thickness=0.5));

  // ============================================================================
  // SUPPLY SPLITTING:  teeMain -> valve_living (direct priority)
  //                    teeMain -> teeCellarRoof -> valve_cellar / valve_roof
  // ============================================================================
  connect(teeMain.port_2, valve_living.port_a)
    annotation(Line(points={{-95,70},{-70,70},{-70,0},{-60,0}}, color={0,127,255}, thickness=0.5));

  connect(teeMain.port_3, teeCellarRoof.port_1)
    annotation(Line(points={{-105,80},{-105,90},{-90,90},{-90,70},{-90,70}}, color={0,127,255}));

  connect(teeCellarRoof.port_2, valve_cellar.port_a)
    annotation(Line(points={{-70,70},{-70,140},{-60,140}}, color={0,127,255}));

  connect(teeCellarRoof.port_3, valve_roof.port_a)
    annotation(Line(points={{-80,80},{-80,-140},{-60,-140}}, color={0,127,255}));

  // ============================================================================
  // VALVE -> HYDRONIC SUBSYSTEM  (straight horizontal connections)
  // ============================================================================
  connect(valve_cellar.port_b, cellar_hydSys.port_a)
    annotation(Line(points={{-40,140},{-20,140}}, color={0,127,255}, thickness=0.5));
  connect(valve_living.port_b, living_hydSys.port_a)
    annotation(Line(points={{-40,0},{-20,0}}, color={0,127,255}, thickness=0.5));
  connect(valve_roof.port_b, roof_hydSys.port_a)
    annotation(Line(points={{-40,-140},{-20,-140}}, color={0,127,255}, thickness=0.5));

  // ============================================================================
  // RETURN PATH MERGING
  // cellar_hydSys + roof_hydSys -> teeMergeCellarRoof
  // teeMergeCellarRoof + living_hydSys -> teeMergeMain -> TReturn -> port_b
  // ============================================================================
  connect(cellar_hydSys.port_b, teeMergeCellarRoof.port_2)
    annotation(Line(points={{20,140},{148,140},{148,-70}}, color={0,127,255}));
  connect(roof_hydSys.port_b, teeMergeCellarRoof.port_3)
    annotation(Line(points={{20,-140},{158,-140},{158,-60}}, color={0,127,255}));
  connect(teeMergeCellarRoof.port_1, teeMergeMain.port_3)
    annotation(Line(points={{168,-70},{175,-70},{175,-60}}, color={0,127,255}));
  connect(living_hydSys.port_b, teeMergeMain.port_2)
    annotation(Line(points={{20,0},{165,0},{165,-70}}, color={0,127,255}));
  connect(teeMergeMain.port_1, TReturn.port_a)
    annotation(Line(points={{185,-70},{185,-195},{186,-195}}, color={0,127,255}, thickness=0.5));
  connect(TReturn.port_b, port_b)
    annotation(Line(points={{206,-195},{206,-210},{-222,-210},{-222,-195}}, color={0,127,255}, thickness=0.5));

  // ============================================================================
  // THERMAL: hydSys -> zone heatPort  (via short diagonal stub)
  // ============================================================================
  connect(cellar_hydSys.port_zone, cellar_zone.heatPort)
    annotation(Line(points={{0,160},{0,168},{80,168},{80,148.4},{88.8,148.4}}, color={191,0,0}));
  connect(living_hydSys.port_zone, living_zone.heatPort)
    annotation(Line(points={{0,20},{0,28},{80,28},{80,8.4},{88.8,8.4}}, color={191,0,0}));
  connect(roof_hydSys.port_zone, roof_zone.heatPort)
    annotation(Line(points={{0,-120},{0,-112},{80,-112},{80,-131.6},{88.8,-131.6}}, color={191,0,0}));

  // ============================================================================
  // CONTROL LOOPS - CELLAR
  // TRefConst_cellar -> PI_cellar.u_s
  // cellar_zone.TZone -> PI_cellar.u_m
  // PI_cellar.y -> valve_cellar.opening
  // TRefConst_cellar -> cellar_zone.TZoneRef
  // ============================================================================
  connect(TRefConst_cellar.y, PI_cellar.u_s)
    annotation(Line(points={{-164,140},{-142,140}}, color={0,0,127}));
  connect(cellar_zone.TZone, PI_cellar.u_m)
    annotation(Line(points={{98,120},{98,110},{-130,110},{-130,128}}, color={0,0,127}));
  connect(PI_cellar.y, valve_cellar.opening)
    annotation(Line(points={{-119,140},{-100,140},{-100,152},{-50,152},{-50,148}}, color={0,0,127}));
  connect(TRefConst_cellar.y, cellar_zone.TZoneRef)
    annotation(Line(points={{-164,140},{-120,140},{-120,178},{130,178},{130,148},{128,148}}, color={0,0,127}));

  // ============================================================================
  // CONTROL LOOPS - LIVING
  // ============================================================================
  connect(TRefConst_living.y, PI_living.u_s)
    annotation(Line(points={{-166,-28},{-142,-28},{-142,0}},
                                                color={0,0,127}));
  connect(living_zone.TZone, PI_living.u_m)
    annotation(Line(points={{98,-20},{98,-30},{-130,-30},{-130,-12}}, color={0,0,127}));
  connect(PI_living.y, valve_living.opening)
    annotation(Line(points={{-119,0},{-100,0},{-100,12},{-50,12},{-50,8}}, color={0,0,127}));
  connect(TRefConst_living.y, living_zone.TZoneRef)
    annotation(Line(points={{-166,-28},{-120,-28},{-120,40},{130,40},{130,8},{128,
          8}},                                                                     color={0,0,127}));

  // ============================================================================
  // CONTROL LOOPS - ROOF
  // ============================================================================
  connect(TRefConst_roof.y, PI_roof.u_s)
    annotation(Line(points={{-162,-140},{-142,-140}}, color={0,0,127}));
  connect(roof_zone.TZone, PI_roof.u_m)
    annotation(Line(points={{98,-160},{98,-170},{-130,-170},{-130,-152}}, color={0,0,127}));
  connect(PI_roof.y, valve_roof.opening)
    annotation(Line(points={{-119,-140},{-100,-140},{-100,-128},{-50,-128},{-50,-132}}, color={0,0,127}));
  connect(TRefConst_roof.y, roof_zone.TZoneRef)
    annotation(Line(points={{-162,-140},{-120,-140},{-120,-100},{130,-100},{130,
          -132},{128,-132}},                                                                       color={0,0,127}));

  // ============================================================================
  // ZONE INPUTS - WINDOW SHADING & SOLAR RADIATION
  // ============================================================================
  connect(WindowShadingConst_cellar.y, cellar_zone.WindowShading)
    annotation(Line(points={{55.8,154},{80,154},{80,119.2},{88.2,119.2}}, color={0,0,127}));
  connect(Q_radiation_cellar.y, cellar_zone.Q_radiation_W)
    annotation(Line(points={{55.8,126},{84,126},{84,129.4},{87.4,129.4}}, color={0,0,127}));

  connect(WindowShadingConst_living.y, living_zone.WindowShading)
    annotation(Line(points={{55.8,14},{80,14},{80,-20.8},{88.2,-20.8}}, color={0,0,127}));
  connect(Q_radiation_living.y, living_zone.Q_radiation_W)
    annotation(Line(points={{55.8,-14},{84,-14},{84,-10.6},{87.4,-10.6}}, color={0,0,127}));

  connect(WindowShadingConst_roof.y, roof_zone.WindowShading)
    annotation(Line(points={{55.8,-126},{80,-126},{80,-160.8},{88.2,-160.8}}, color={0,0,127}));
  connect(Q_radiation_roof.y, roof_zone.Q_radiation_W)
    annotation(Line(points={{55.8,-154},{84,-154},{84,-150.6},{87.4,-150.6}}, color={0,0,127}));

  // ============================================================================
  // ZONE INPUTS - OCCUPANCY  (routed along top)
  // ============================================================================
  connect(nPersons_cellar, cellar_zone.nPersons)
    annotation(Line(points={{-222,185},{130,185},{130,152.4},{132.8,152.4}},
                                                                           color={0,0,127}));
  connect(nPersons_living, living_zone.nPersons)
    annotation(Line(points={{-222,165},{-10,165},{-10,54},{132,54},{132,12.4},{
          132.8,12.4}},                                                                    color={0,0,127}));
  connect(nPersons_roof, roof_zone.nPersons)
    annotation(Line(points={{-222,145},{-12,145},{-12,-46},{136,-46},{136,
          -127.6},{132.8,-127.6}},                                                               color={0,0,127}));

  // ============================================================================
  // ZONE INPUTS - APPLIANCE LOADS  (from right side)
  // ============================================================================
  connect(P_appliances_cellar_W, cellar_zone.P_appliances_W)
    annotation(Line(points={{250,140},{220,140},{220,152.8},{87.2,152.8}}, color={0,0,127}));
  connect(P_appliances_living_W, living_zone.P_appliances_W)
    annotation(Line(points={{250,0},{220,0},{220,12.8},{87.2,12.8}}, color={0,0,127}));
  connect(P_appliances_roof_W, roof_zone.P_appliances_W)
    annotation(Line(points={{250,-140},{220,-140},{220,-127.2},{87.2,-127.2}}, color={0,0,127}));

  // ============================================================================
  // ZONE INPUTS - SOLAR RADIATION  (routed along bottom)
  // ============================================================================
  connect(solRad_roof, roof_zone.solRadIn)
    annotation(Line(points={{-222,-145},{-14,-145},{-14,-162},{110,-162},{110,-161}}, color={0,0,127}));
  connect(solRad_living, living_zone.solRadIn)
    annotation(Line(points={{-222,-165},{-16,-165},{-16,-40},{112,-40},{112,-21}}, color={0,0,127}));
  connect(solRad_cellar, cellar_zone.solRadIn)
    annotation(Line(points={{-222,-185},{-18,-185},{-18,100},{110,100},{110,118}}, color={0,0,127}));

  // ============================================================================
  // OUTPUTS - ZONE TEMPERATURES & VALVE POSITIONS
  // ============================================================================
  connect(cellar_zone.TZone, TZone_cellar)
    annotation(Line(points={{98,120},{218,120},{218,140},{240,140}}, color={0,0,127}));
  connect(PI_cellar.y, valve_cellar_opening)
    annotation(Line(points={{-119,140},{-110,140},{-110,190},{230,190},{230,115},{240,115}}, color={0,0,127}));

  connect(living_zone.TZone, TZone_living)
    annotation(Line(points={{98,-20},{218,-20},{218,0},{240,0}}, color={0,0,127}));
  connect(PI_living.y, valve_living_opening)
    annotation(Line(points={{-119,0},{-112,0},{-112,-50},{230,-50},{230,-25},{240,-25}}, color={0,0,127}));

  connect(roof_zone.TZone, TZone_roof)
    annotation(Line(points={{98,-160},{218,-160},{218,-140},{240,-140}}, color={0,0,127}));
  connect(PI_roof.y, valve_roof_opening)
    annotation(Line(points={{-119,-140},{-112,-140},{-112,-200},{230,-200},{230,-165},{240,-165}}, color={0,0,127}));

  // ============================================================================
  // FLOW RATE APPROXIMATION
  // ============================================================================
  connect(PI_cellar.y, qv_cellar_approx.u)
    annotation(Line(points={{-119,140},{152,140}}, color={0,0,127}));
  connect(PI_living.y, qv_living_approx.u)
    annotation(Line(points={{-119,0},{152,0}}, color={0,0,127}));
  connect(PI_roof.y, qv_roof_approx.u)
    annotation(Line(points={{-119,-140},{152,-140}}, color={0,0,127}));

  connect(qv_cellar_approx.y, qvSum.u1)
    annotation(Line(points={{170.8,140},{190,140},{190,-62},{193,-62}}, color={0,0,127}));
  connect(qv_living_approx.y, qvSum.u2)
    annotation(Line(points={{170.8,0},{190,0},{190,-70},{193,-70}}, color={0,0,127}));
  connect(qv_roof_approx.y, qvSum.u3)
    annotation(Line(points={{170.8,-140},{190,-140},{190,-78},{193,-78}}, color={0,0,127}));
  connect(qvSum.y, qvRef)
    annotation(Line(points={{216,-70},{230,-70},{230,-200},{240,-200}}, color={0,0,127}));

annotation(
  Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
    graphics={
      Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0},
        fillColor={255,255,255}, fillPattern=FillPattern.Solid),
      Polygon(points={{-70,60},{0,85},{70,60},{70,40},{-70,40},{-70,60}},
        lineColor={0,0,255}, fillColor={200,220,255}, fillPattern=FillPattern.Solid, lineThickness=1),
      Rectangle(extent={{-70,40},{70,0}}, lineColor={0,128,0},
        fillColor={220,255,220}, fillPattern=FillPattern.Solid, lineThickness=1),
      Rectangle(extent={{-70,0},{70,-40}}, lineColor={139,69,19},
        fillColor={255,235,205}, fillPattern=FillPattern.Solid, lineThickness=1),
      Line(points={{-90,-40},{90,-40}}, color={100,100,100}, thickness=2),
      Text(extent={{-60,72},{60,62}}, textString="ROOF", textColor={0,0,200},
        textStyle={TextStyle.Bold}, fontSize=10),
      Text(extent={{-60,55},{60,45}}, textString="Office/Attic", textColor={0,0,150}, fontSize=7),
      Text(extent={{-60,32},{60,22}}, textString="LIVING", textColor={0,128,0},
        textStyle={TextStyle.Bold}, fontSize=10),
      Text(extent={{-60,15},{60,5}}, textString="Main Floor", textColor={0,100,0}, fontSize=7),
      Text(extent={{-60,-8},{60,-18}}, textString="CELLAR", textColor={139,69,19},
        textStyle={TextStyle.Bold}, fontSize=10),
      Text(extent={{-60,-25},{60,-35}}, textString="Basement", textColor={100,50,0}, fontSize=7),
      Text(extent={{-90,100},{90,85}}, textString="%name", textColor={0,0,0}, textStyle={TextStyle.Bold}),
      Text(extent={{-90,-50},{90,-65}}, textString="Living Priority", textColor={0,128,0},
        textStyle={TextStyle.Bold}, fontSize=9)}),

  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-230,-215},{260,215}})),

  Documentation(info="<html>
<h4>Three-Zone Building Model - Clean Layout</h4>
<p>Three clearly separated horizontal zone rows:</p>
<ul>
<li><b>CELLAR</b> (y=+140): top row</li>
<li><b>LIVING</b> (y=0):   middle row (priority zone)</li>
<li><b>ROOF</b>   (y=-140): bottom row</li>
</ul>
<p>Column layout (left to right): TRefConst | PI | Valve | HydSys | Zone | qv_approx</p>
</html>"));
end ThreeZoneBuilding_optimized;
