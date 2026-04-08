within CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis;
model ThreeZoneBuilding_PHiL_basic
  "PHiL wrapper for ThreeZoneBuilding - CoSES Lab Compatible - Corrected Flow"

  // ============================================================================
  // PHiL INPUT SIGNALS (from CoSES Lab)
  // ============================================================================

  Modelica.Blocks.Interfaces.RealInput STM_HCVLaM_degC(start=50)
    "Temperature Heating Circuit after Mixing Valve [degC]"
    annotation (Placement(transformation(extent={{-260,92},{-220,132}}),
        iconTransformation(extent={{-260,92},{-220,132}})));

  Modelica.Blocks.Interfaces.RealInput SFW_HCRLbM_l_per_min(start=6)
    "Flow Water Heating Circuit before Mixing valve [L/min]"
    annotation (Placement(transformation(extent={{-260,28},{-220,68}}),
        iconTransformation(extent={{-260,28},{-220,68}})));

  Modelica.Blocks.Interfaces.RealInput T_ambient_degC(start=5)
    "Outdoor ambient temperature [degC] - reference only, not wired to building parameter"
    annotation (Placement(transformation(extent={{-260,-8},{-220,32}}),
        iconTransformation(extent={{-260,-8},{-220,32}})));

  // ============================================================================
  // PHiL OUTPUT SIGNALS (to CoSES Lab)
  // ============================================================================

  Modelica.Blocks.Interfaces.RealOutput T_roomIs_degC
    "Living zone temperature [degC]"
    annotation (Placement(transformation(extent={{220,104},{240,124}}),
        iconTransformation(extent={{220,104},{240,124}})));

  Modelica.Blocks.Interfaces.RealOutput T_cellarIs_degC
    "Cellar zone temperature [degC]"
    annotation (Placement(transformation(extent={{222,72},{242,92}}),
        iconTransformation(extent={{222,72},{242,92}})));

  Modelica.Blocks.Interfaces.RealOutput T_roofIs_degC
    "Roof zone temperature [degC]"
    annotation (Placement(transformation(extent={{222,40},{242,60}}),
        iconTransformation(extent={{222,40},{242,60}})));

  Modelica.Blocks.Interfaces.RealOutput STM_HCRL_Set_degC
    "Return water temperature setpoint [degC]"
    annotation (Placement(transformation(extent={{222,16},{242,36}}),
        iconTransformation(extent={{222,16},{242,36}})));

  Modelica.Blocks.Interfaces.RealOutput SFW_HCRLbM_Set_l_per_min
    "Volume flow rate setpoint [L/min]"
    annotation (Placement(transformation(extent={{224,-6},{244,14}}),
        iconTransformation(extent={{224,-6},{244,14}})));

  // ============================================================================
  // BUILDING PARAMETERS
  // ============================================================================

  parameter Modelica.Units.SI.Area AZone_cellar  = 80  "Cellar floor area [m2]";
  parameter Modelica.Units.SI.Area AZone_living  = 100 "Living zone floor area [m2]";
  parameter Modelica.Units.SI.Area AZone_roof    = 60  "Roof zone floor area [m2]";

  parameter Modelica.Units.SI.Length hZone_cellar = 2.2 "Cellar height [m]";
  parameter Modelica.Units.SI.Length hZone_living = 2.5 "Living zone height [m]";
  parameter Modelica.Units.SI.Length hZone_roof   = 2.3 "Roof zone height [m]";

  parameter Modelica.Units.SI.Temperature TZoneInit_cellar = 288.15 "Cellar initial temp (15 degC)";
  parameter Modelica.Units.SI.Temperature TZoneInit_living = 294.15 "Living initial temp (21 degC)";
  parameter Modelica.Units.SI.Temperature TZoneInit_roof   = 289.15 "Roof initial temp (16 degC)";

  parameter Modelica.Units.SI.Temperature TRef_cellar = 288.15 "Cellar setpoint (15 degC)";
  parameter Modelica.Units.SI.Temperature TRef_living = 294.15 "Living setpoint (21 degC)";
  parameter Modelica.Units.SI.Temperature TRef_roof   = 294.15 "Roof setpoint (21 degC)";

  parameter Modelica.Units.SI.Temperature TOutdoor = 278.15 "Outdoor temp (5 degC)";

  parameter Boolean cellarHeat = true "If true, cellar is heated";
  parameter Boolean roofHeat   = true "If true, roof is heated";

  // ============================================================================
  // MEDIUM
  // ============================================================================

  replaceable package Medium = IBPSA.Media.Water
    constrainedby Modelica.Media.Interfaces.PartialMedium
    annotation (choicesAllMatching=true);

  // ============================================================================
  // SYSTEM
  // ============================================================================

  inner Modelica.Fluid.System system(
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    massDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    annotation (Placement(transformation(extent={{-200,140},{-180,160}})));

  // ============================================================================
  // FLUID BOUNDARIES
  // ============================================================================

  Modelica.Fluid.Sources.MassFlowSource_T supply(
    redeclare package Medium = Medium,
    use_m_flow_in = true,
    use_T_in     = true,
    T            = 333.15,
    nPorts       = 1)
    "Hot water supply - controlled temperature and flow rate"
    annotation (Placement(transformation(extent={{-140,54},{-120,74}})));

  Modelica.Fluid.Sources.Boundary_pT return_sink(
    redeclare package Medium = Medium,
    p      = 200000,
    T      = 313.15,
    nPorts = 1)
    "Return water sink - pressure reference"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-130,-60})));

  Modelica.Fluid.Sensors.TemperatureTwoPort TReturn(
    redeclare package Medium = Medium)
    "Return temperature sensor"
    annotation (Placement(transformation(extent={{-80,-70},{-100,-50}})));

  // ============================================================================
  // UNIT CONVERSIONS - SUPPLY SIDE
  // ============================================================================

  Modelica.Blocks.Math.Add toKelvin(k1=1, k2=1)
    "Convert supply temp from degC to K"
    annotation (Placement(transformation(extent={{-180,76},{-160,96}})));

  Modelica.Blocks.Sources.Constant kelvinOffset(k=273.15)
    "Kelvin offset (+273.15)"
    annotation (Placement(transformation(extent={{-206,56},{-196,66}})));

  Modelica.Blocks.Math.Gain flowConvert(k=1/60)
    "Convert L/min to kg/s"
    annotation (Placement(transformation(extent={{-180,38},{-160,58}})));

  // ============================================================================
  // UNIT CONVERSIONS - OUTPUT SIDE (K -> degC)
  // ============================================================================

  Modelica.Blocks.Math.Add toC_living(k2=-1)
    "Living zone K to degC"
    annotation (Placement(transformation(extent={{150,100},{170,120}})));

  Modelica.Blocks.Math.Add toC_cellar(k2=-1)
    "Cellar zone K to degC"
    annotation (Placement(transformation(extent={{150,68},{170,88}})));

  Modelica.Blocks.Math.Add toC_roof(k2=-1)
    "Roof zone K to degC"
    annotation (Placement(transformation(extent={{150,36},{170,56}})));

  Modelica.Blocks.Math.Add toC_return(k2=-1)
    "Return temp K to degC"
    annotation (Placement(transformation(extent={{150,12},{170,32}})));

  Modelica.Blocks.Sources.Constant celsiusOffset(k=-273.15)
    "Offset for K to degC conversion (-273.15)"
    annotation (Placement(transformation(extent={{110,-110},{130,-90}})));

  // ============================================================================
  // THE BUILDING MODEL
  // ============================================================================

  CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis.ThreeZoneBuilding_basic building(
    redeclare package Medium   = Medium,
    AZone_cellar   = AZone_cellar,
    AZone_living   = AZone_living,
    AZone_roof     = AZone_roof,
    hZone_cellar   = hZone_cellar,
    hZone_living   = hZone_living,
    hZone_roof     = hZone_roof,
    TZoneInit_cellar = TZoneInit_cellar,
    TZoneInit_living = TZoneInit_living,
    TZoneInit_roof   = TZoneInit_roof,
    TRef_cellar    = TRef_cellar,
    TRef_living    = TRef_living,
    TRef_roof      = TRef_roof,
    TOutdoor       = TOutdoor,
    cellarHeat     = cellarHeat,
    roofHeat       = roofHeat)
    "Three-zone building model"
    annotation (Placement(transformation(extent={{-20,-56},{100,104}})));

  // ============================================================================
  // INTERNAL GAINS INPUTS
  // ============================================================================

  Modelica.Blocks.Interfaces.RealInput nPersons_living_in(start=2)
    "Number of persons in living zone"
    annotation (Placement(
      transformation(extent={{-260,-42},{-220,-2}}),
      iconTransformation(extent={{-280,-20},{-240,20}})));

  Modelica.Blocks.Interfaces.RealInput nPersons_cellar_in(start=0)
    "Number of persons in cellar"
    annotation (Placement(
      transformation(extent={{-260,-80},{-220,-40}}),
      iconTransformation(extent={{-280,-80},{-240,-40}})));

  Modelica.Blocks.Interfaces.RealInput nPersons_roof_in(start=0)
    "Number of persons in roof zone"
    annotation (Placement(
      transformation(extent={{-260,-122},{-220,-82}}),
      iconTransformation(extent={{-280,40},{-240,80}})));

  Modelica.Blocks.Interfaces.RealInput P_appliances_living_W_in(start=200)
    "Living zone appliance power [W]"
    annotation (Placement(
      transformation(extent={{258,-44},{218,-4}}),
      iconTransformation(extent={{280,-20},{240,20}})));

  Modelica.Blocks.Interfaces.RealInput P_appliances_cellar_W_in(start=50)
    "Cellar appliance power [W]"
    annotation (Placement(
      transformation(extent={{258,-80},{218,-40}}),
      iconTransformation(extent={{280,-80},{240,-40}})));

  Modelica.Blocks.Interfaces.RealInput P_appliances_roof_W_in(start=50)
    "Roof zone appliance power [W]"
    annotation (Placement(
      transformation(extent={{260,-120},{220,-80}}),
      iconTransformation(extent={{280,40},{240,80}})));

equation

  // ============================================================================
  // SFW PASSTHROUGH (arithmetic - no block needed)
  // NOTE: T_ambient_degC is a reference input only; TOutdoor is a parameter
  //       in ThreeZoneBuilding_basic and cannot be driven dynamically here.
  // ============================================================================
  SFW_HCRLbM_Set_l_per_min = SFW_HCRLbM_l_per_min;

  // ============================================================================
  // 1. SUPPLY TEMPERATURE PATH
  //    STM_HCVLaM_degC -> toKelvin -> supply.T_in
  // ============================================================================
  connect(STM_HCVLaM_degC, toKelvin.u1)
    annotation (Line(
      points={{-240,112},{-192,112},{-192,92},{-182,92}},
      color={0,0,127}));

  connect(kelvinOffset.y, toKelvin.u2)
    annotation (Line(
      points={{-195.5,61},{-192,61},{-192,80},{-182,80}},
      color={0,0,127}));

  connect(toKelvin.y, supply.T_in)
    annotation (Line(
      points={{-159,86},{-152,86},{-152,68},{-142,68}},
      color={0,0,127}));

  // ============================================================================
  // 2. SUPPLY FLOW PATH
  //    SFW_HCRLbM_l_per_min -> flowConvert -> supply.m_flow_in
  // ============================================================================
  connect(SFW_HCRLbM_l_per_min, flowConvert.u)
    annotation (Line(
      points={{-240,48},{-182,48}},
      color={0,0,127}));

  connect(flowConvert.y, supply.m_flow_in)
    annotation (Line(
      points={{-159,48},{-150,48},{-150,80},{-140,80},{-140,72}},
      color={0,0,127}));

  // ============================================================================
  // 3. FLUID PATH
  //    supply -> building.port_a -> building -> building.port_b
  //    -> TReturn -> return_sink
  // ============================================================================
  connect(supply.ports[1], building.port_a)
    annotation (Line(
      points={{-120,64},{-70,64},{-70,68.4444},{-20,68.4444}},
      color={0,127,255},
      thickness=0.5));

  connect(building.port_b, TReturn.port_a)
    annotation (Line(
      points={{-20,1.77778},{-74,1.77778},{-74,-60},{-80,-60}},
      color={0,127,255},
      thickness=0.5));

  connect(TReturn.port_b, return_sink.ports[1])
    annotation (Line(
      points={{-100,-60},{-120,-60}},
      color={0,127,255},
      thickness=0.5));

  // ============================================================================
  // 4. OCCUPANCY INPUTS
  //    nPersons_*_in -> building.nPersons_*
  //    Routes go below supply/TReturn blocks, then rise to building left ports
  // ============================================================================
  connect(nPersons_living_in, building.nPersons_living)
    annotation (Line(
      points={{-240,-22},{-35,-22},{-35,90.6667},{-71,90.6667}},
      color={0,0,127}));

  connect(nPersons_cellar_in, building.nPersons_cellar)
    annotation (Line(
      points={{-240,-60},{-240,-75},{-40,-75},{-40,108.444},{-71,108.444}},
      color={0,0,127}));

  connect(nPersons_roof_in, building.nPersons_roof)
    annotation (Line(
      points={{-240,-102},{-40,-102},{-40,72.8889},{-71,72.8889}},
      color={0,0,127}));

  // ============================================================================
  // 5. APPLIANCE POWER INPUTS
  //    P_appliances_*_W_in -> building.P_appliances_*_W
  //    Routes come from right edge, go above building bottom, reach right ports
  // ============================================================================
  connect(P_appliances_living_W_in, building.P_appliances_living_W)
    annotation (Line(
      points={{238,-24},{190,-24},{190,90.6667},{151,90.6667}},
      color={0,0,127}));

  connect(P_appliances_cellar_W_in, building.P_appliances_cellar_W)
    annotation (Line(
      points={{238,-60},{200,-60},{200,108.444},{151,108.444}},
      color={0,0,127}));

  connect(P_appliances_roof_W_in, building.P_appliances_roof_W)
    annotation (Line(
      points={{240,-100},{210,-100},{210,72.8889},{151,72.8889}},
      color={0,0,127}));

  // ============================================================================
  // 6. ZONE TEMPERATURE OUTPUTS  ->  K to degC conversion  ->  output ports
  //    building.TZone_* -> toC_* -> T_*Is_degC
  // ============================================================================

  // celsiusOffset shared bus: routes to u2 of all four Add blocks
  connect(celsiusOffset.y, toC_living.u2)
    annotation (Line(
      points={{131,-100},{140,-100},{140,105},{148,105}},
      color={0,0,127}));

  connect(celsiusOffset.y, toC_cellar.u2)
    annotation (Line(
      points={{131,-100},{140,-100},{140,73},{148,73}},
      color={0,0,127}));

  connect(celsiusOffset.y, toC_roof.u2)
    annotation (Line(
      points={{131,-100},{140,-100},{140,41},{148,41}},
      color={0,0,127}));

  connect(celsiusOffset.y, toC_return.u2)
    annotation (Line(
      points={{131,-100},{140,-100},{140,17},{148,17}},
      color={0,0,127}));

  // Zone temperature u1 inputs from building right edge
  connect(building.TZone_living, toC_living.u1)
    annotation (Line(
      points={{103,41.7778},{120,41.7778},{120,115},{148,115}},
      color={0,0,127}));

  connect(building.TZone_cellar, toC_cellar.u1)
    annotation (Line(
      points={{103,86.2222},{120,86.2222},{120,83},{148,83}},
      color={0,0,127}));

  connect(building.TZone_roof, toC_roof.u1)
    annotation (Line(
      points={{103,-2.66667},{120,-2.66667},{120,51},{148,51}},
      color={0,0,127}));

  // Return temperature from TReturn sensor
  connect(TReturn.T, toC_return.u1)
    annotation (Line(
      points={{-90,-49},{-90,-75},{130,-75},{130,27},{148,27}},
      color={0,0,127}));

  // Outputs: conversion result -> PHiL output port
  connect(toC_living.y, T_roomIs_degC)
    annotation (Line(
      points={{171,110},{200,110},{200,114},{230,114}},
      color={0,0,127}));

  connect(toC_cellar.y, T_cellarIs_degC)
    annotation (Line(
      points={{171,78},{200,78},{200,82},{232,82}},
      color={0,0,127}));

  connect(toC_roof.y, T_roofIs_degC)
    annotation (Line(
      points={{171,46},{200,46},{200,50},{232,50}},
      color={0,0,127}));

  connect(toC_return.y, STM_HCRL_Set_degC)
    annotation (Line(
      points={{171,22},{200,22},{200,26},{232,26}},
      color={0,0,127}));

  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-260,-200},{260,200}}),
      graphics={
        Rectangle(
          extent={{-260,200},{260,-200}},
          lineColor={135,206,235},
          fillColor={135,206,235},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-260,-120},{260,-200}},
          lineColor={34,139,34},
          fillColor={34,139,34},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{180,180},{220,140}},
          lineColor={255,215,0},
          fillColor={255,215,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-140,-120},{140,40}},
          lineColor={101,67,33},
          fillColor={255,228,196},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{-160,40},{0,130},{160,40},{-160,40}},
          lineColor={101,67,33},
          fillColor={178,34,34},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Rectangle(
          extent={{-120,-120},{120,-150}},
          lineColor={105,105,105},
          fillColor={169,169,169},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-100,-125},{100,-145}},
          textColor={255,255,255},
          textString="Cellar",
          textStyle={TextStyle.Bold}),
        Rectangle(
          extent={{-120,-100},{120,20}},
          lineColor={0,0,0},
          fillColor={255,250,205},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-100,-30},{100,-60}},
          textColor={0,0,0},
          textString="Living",
          textStyle={TextStyle.Bold}),
        Polygon(
          points={{-100,30},{0,100},{100,30},{-100,30}},
          lineColor={0,0,0},
          fillColor={255,218,185},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-60,85},{60,50}},
          textColor={0,0,0},
          textString="Roof",
          textStyle={TextStyle.Bold}),
        Rectangle(
          extent={{-18,-100},{18,-50}},
          lineColor={101,67,33},
          fillColor={139,69,19},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-20},{-55,5}},
          lineColor={0,0,139},
          fillColor={173,216,230},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{55,-20},{100,5}},
          lineColor={0,0,139},
          fillColor={173,216,230},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{70,90},{90,140}},
          lineColor={101,67,33},
          fillColor={178,34,34},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-258,178},{-162,158}},
          textColor={255,0,0},
          textString="T_supply",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-258,138},{-162,118}},
          textColor={0,0,255},
          textString="Flow",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-258,98},{-162,78}},
          textColor={0,128,0},
          textString="T_ambient",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-258,68},{-170,48}},
          textColor={128,0,128},
          textString="nPers_roof",
          textStyle={TextStyle.Italic}),
        Text(
          extent={{-258,8},{-170,-12}},
          textColor={128,0,128},
          textString="nPers_living",
          textStyle={TextStyle.Italic}),
        Text(
          extent={{-258,-52},{-170,-72}},
          textColor={128,0,128},
          textString="nPers_cellar",
          textStyle={TextStyle.Italic}),
        Text(
          extent={{162,178},{258,158}},
          textColor={238,46,47},
          textString="T_living",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{162,138},{258,118}},
          textColor={238,46,47},
          textString="T_cellar",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{162,98},{258,78}},
          textColor={238,46,47},
          textString="T_roof",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{170,68},{258,48}},
          textColor={255,128,0},
          textString="P_app_roof",
          textStyle={TextStyle.Italic}),
        Text(
          extent={{170,8},{258,-12}},
          textColor={255,128,0},
          textString="P_app_living",
          textStyle={TextStyle.Italic}),
        Text(
          extent={{170,-52},{258,-72}},
          textColor={255,128,0},
          textString="P_app_cellar",
          textStyle={TextStyle.Italic}),
        Text(
          extent={{162,-112},{258,-132}},
          textColor={0,0,255},
          textString="T_return",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{162,-152},{258,-172}},
          textColor={0,128,255},
          textString="Flow_set",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-200,198},{200,178}},
          textColor={0,0,255},
          textString="%name",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-200,-175},{200,-195}},
          textColor={0,0,0},
          textString="PHiL Building - Variable Internal Gains")}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-240,-120},{240,180}})),
    experiment(
      StartTime=0,
      StopTime=259200,
      Interval=60,
      Tolerance=1e-06),
    Documentation(info="<html>
<h4>ThreeZoneBuilding_PHiL_basic - Fully Annotated</h4>
<li>
Apr 8, 2026, by Karthik Murugesan:<br/>
Replaced all direct equation assignments (nPersons, P_appliances, zone temp outputs)
with connect() statements carrying Line annotations for full diagram visibility.
Added toC_living/cellar/roof/return Add blocks and shared celsiusOffset constant
for graphical K-to-degC conversion chain. Fixed STM_HCVLaM_degC start=60 to
start=50 to match actual VeriStand supply temperature.
Note: T_ambient_degC remains a reference-only input; TOutdoor is a parameter
in ThreeZoneBuilding_basic and cannot be driven dynamically without model change.
<br/>
</li>
<li>
Feb 28, 2026, by Karthik Murugesan
</li>
<p>PHiL wrapper for three-zone building with organized connectors.</p>
<h5>Inputs (Left side):</h5>
<ul>
<li><b>T_supply</b> - Supply water temperature [degC]</li>
<li><b>Flow</b> - Volume flow rate [L/min]</li>
<li><b>T_ambient</b> - Outdoor temperature [degC] (reference only)</li>
<li><i>nPers_*</i> - Occupancy per zone (purple)</li>
</ul>
<h5>Inputs (Right side):</h5>
<ul>
<li><i>P_app_*</i> - Appliance power per zone [W] (orange)</li>
</ul>
<h5>Outputs (Right side):</h5>
<ul>
<li><b>T_living, T_cellar, T_roof</b> - Zone temperatures [degC]</li>
<li><b>T_return</b> - Return water temperature [degC]</li>
<li><b>Flow_set</b> - Flow setpoint passthrough [L/min]</li>
</ul>
</html>"));
end ThreeZoneBuilding_PHiL_basic;
