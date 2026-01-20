within CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis;
model ThreeZoneBuilding_PHiL_basic
  "PHiL wrapper for ThreeZoneBuilding - CoSES Lab Compatible - CORRECTED FLOW"

  // ============================================================================
  // PHiL INPUT SIGNALS (from CoSES Lab)
  // ============================================================================

  Modelica.Blocks.Interfaces.RealInput STM_HCVLaM_degC(start=60)
    "Temperature Heating Circuit after Mixing Valve [°C]"
    annotation (Placement(transformation(extent={{-260,92},{-220,132}}),
        iconTransformation(extent={{-260,92},{-220,132}})));

  Modelica.Blocks.Interfaces.RealInput SFW_HCRLbM_l_per_min(start=6)
    "Flow Water Heating Circuit before Mixing valve [L/min]"
    annotation (Placement(transformation(extent={{-260,28},{-220,68}}),
        iconTransformation(extent={{-260,28},{-220,68}})));

  Modelica.Blocks.Interfaces.RealInput T_ambient_degC(start=5)
    "Outdoor ambient temperature [°C]"
    annotation (Placement(transformation(extent={{-260,-8},{-220,32}}),
        iconTransformation(extent={{-260,-8},{-220,32}})));

  // ============================================================================
  // PHiL OUTPUT SIGNALS (to CoSES Lab)
  // ============================================================================

  Modelica.Blocks.Interfaces.RealOutput T_roomIs_degC
    "Living zone temperature [°C]"
    annotation (Placement(transformation(extent={{220,104},{240,124}}),
        iconTransformation(extent={{220,104},{240,124}})));

  Modelica.Blocks.Interfaces.RealOutput T_cellarIs_degC
    "Cellar zone temperature [°C]"
    annotation (Placement(transformation(extent={{222,72},{242,92}}),
        iconTransformation(extent={{222,72},{242,92}})));

  Modelica.Blocks.Interfaces.RealOutput T_roofIs_degC
    "Roof zone temperature [°C]"
    annotation (Placement(transformation(extent={{222,40},{242,60}}),
        iconTransformation(extent={{222,40},{242,60}})));

  Modelica.Blocks.Interfaces.RealOutput STM_HCRL_Set_degC
    "Return water temperature setpoint [°C]"
    annotation (Placement(transformation(extent={{222,16},{242,36}}),
        iconTransformation(extent={{222,16},{242,36}})));

  Modelica.Blocks.Interfaces.RealOutput SFW_HCRLbM_Set_l_per_min
    "Volume flow rate setpoint [L/min]"
    annotation (Placement(transformation(extent={{224,-6},{244,14}}),
        iconTransformation(extent={{224,-6},{244,14}})));

  // ============================================================================
  // BUILDING PARAMETERS
  // ============================================================================

  parameter Modelica.Units.SI.Area AZone_cellar = 80 "Cellar floor area [m²]";
  parameter Modelica.Units.SI.Area AZone_living = 100 "Living zone floor area [m²]";
  parameter Modelica.Units.SI.Area AZone_roof = 60 "Roof zone floor area [m²]";

  parameter Modelica.Units.SI.Length hZone_cellar = 2.2 "Cellar height [m]";
  parameter Modelica.Units.SI.Length hZone_living = 2.5 "Living zone height [m]";
  parameter Modelica.Units.SI.Length hZone_roof = 2.3 "Roof zone height [m]";

  // Initial temperatures (warmer to avoid cold transients)
  parameter Modelica.Units.SI.Temperature TZoneInit_cellar = 288.15 "Cellar initial (15°C)";
  parameter Modelica.Units.SI.Temperature TZoneInit_living = 294.15 "Living initial (21°C)";
  parameter Modelica.Units.SI.Temperature TZoneInit_roof = 289.15 "Roof initial (16°C)";

  // Setpoints
  parameter Modelica.Units.SI.Temperature TRef_cellar = 288.15 "Cellar setpoint (15°C)";
  parameter Modelica.Units.SI.Temperature TRef_living = 294.15 "Living setpoint (21°C)";
  parameter Modelica.Units.SI.Temperature TRef_roof = 294.15 "Roof setpoint (21°C)";

  parameter Modelica.Units.SI.Temperature TOutdoor = 278.15 "Outdoor temp (5°C)";

  parameter Boolean cellarHeat = true "If true, cellar is heated";
  parameter Boolean roofHeat = true "If true, roof is heated";

  // ============================================================================
  // MEDIUM - Use ConstantPropertyLiquidWater to avoid IF97 temp limits!
  // ============================================================================

  replaceable package Medium = IBPSA.Media.Water
    constrainedby Modelica.Media.Interfaces.PartialMedium
    annotation (choicesAllMatching=true); // 11 Jan 19.00 IBPSA.Media.Water // 25 Dec 08.00 Modelica.Media.Water.ConstantPropertyLiquidWater
// 25 Dec 09.00 Modelica.Media.Water.StandardWater

  // ============================================================================
  // SYSTEM
  // ============================================================================

  inner Modelica.Fluid.System system(
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    massDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    annotation (Placement(transformation(extent={{-200,140},{-180,160}})));

  // ============================================================================
  // FLUID BOUNDARIES - CORRECTED!
  // ============================================================================
  // KEY FIX: Supply uses MassFlowSource_T to PUSH water INTO the building
  //          Return uses Boundary_pT as pressure reference for water to EXIT
  // ============================================================================

  // Supply boundary - PUSHES hot water at controlled temperature and flow rate
  Modelica.Fluid.Sources.MassFlowSource_T supply(
    redeclare package Medium = Medium,
    use_m_flow_in = true,
    use_T_in = true,
    T = 333.15,
    nPorts = 1)
    "Hot water supply - controls BOTH flow rate and temperature"
    annotation (Placement(transformation(extent={{-140,54},{-120,74}})));

  // Return boundary - provides pressure reference for return water to exit
  Modelica.Fluid.Sources.Boundary_pT return_sink(
    redeclare package Medium = Medium,
    p = 200000,
    T = 313.15,
    nPorts = 1)
    "Return water sink - pressure reference only"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-130,-60})));

  // Return temperature sensor
  Modelica.Fluid.Sensors.TemperatureTwoPort TReturn(
    redeclare package Medium = Medium)
    "Measure return temperature"
    annotation (Placement(transformation(extent={{-80,-70},{-100,-50}})));

  // ============================================================================
  // UNIT CONVERSION BLOCKS
  // ============================================================================

  // Temperature: °C to K
  Modelica.Blocks.Math.Add toKelvin(k1=1, k2=1)
    "Convert °C to K: output = input + 273.15"
    annotation (Placement(transformation(extent={{-180,76},{-160,96}})));

  Modelica.Blocks.Sources.Constant kelvinOffset(k=273.15)
    "Kelvin offset"
    annotation (Placement(transformation(extent={{-206,56},{-196,66}})));

  // Flow: L/min to kg/s (POSITIVE - pushes water INTO building)
  Modelica.Blocks.Math.Gain flowConvert(k=1/60)
    "Convert L/min to kg/s"
    annotation (Placement(transformation(extent={{-180,38},{-160,58}})));

  // ============================================================================
  // THE BUILDING MODEL
  // ============================================================================

  CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis.ThreeZoneBuilding_basic building(
    redeclare package Medium = Medium,
    AZone_cellar = AZone_cellar,
    AZone_living = AZone_living,
    AZone_roof = AZone_roof,
    hZone_cellar = hZone_cellar,
    hZone_living = hZone_living,
    hZone_roof = hZone_roof,
    TZoneInit_cellar = TZoneInit_cellar,
    TZoneInit_living = TZoneInit_living,
    TZoneInit_roof = TZoneInit_roof,
    TRef_cellar = TRef_cellar,
    TRef_living = TRef_living,
    TRef_roof = TRef_roof,
    TOutdoor = TOutdoor,
    cellarHeat = cellarHeat,
    roofHeat = roofHeat)
    "Three-zone building model"
    annotation (Placement(transformation(extent={{-20,-56},{100,104}})));

  // ============================================================================
  // INTERNAL GAINS INPUTS (organized on icon: occupancy=left, appliances=right)
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
  // INTERNAL GAINS PASS-THROUGH (Input -> Input requires equation, not connect)
  // ============================================================================
  building.nPersons_cellar = nPersons_cellar_in;
  building.nPersons_living = nPersons_living_in;
  building.nPersons_roof = nPersons_roof_in;

  building.P_appliances_cellar_W = P_appliances_cellar_W_in;
  building.P_appliances_living_W = P_appliances_living_W_in;
  building.P_appliances_roof_W = P_appliances_roof_W_in;

  // INPUT UNIT CONVERSIONS
  // ============================================================================

  // Supply temperature: °C → K
  connect(STM_HCVLaM_degC, toKelvin.u1)
    annotation (Line(points={{-240,112},{-192,112},{-192,92},{-182,92}},
                                                                       color={0,0,127}));
  connect(kelvinOffset.y, toKelvin.u2)
    annotation (Line(points={{-195.5,61},{-192,61},{-192,80},{-182,80}}, color={0,0,127}));

  // Flow rate: L/min → kg/s
  connect(SFW_HCRLbM_l_per_min, flowConvert.u)
    annotation (Line(points={{-240,48},{-182,48}},                     color={0,0,127}));

  // ============================================================================
  // FLUID BOUNDARY CONTROL - CORRECTED!
  // ============================================================================

  // Supply temperature (to MassFlowSource_T)
  connect(toKelvin.y, supply.T_in)
    annotation (Line(points={{-159,86},{-152,86},{-152,68},{-142,68}}, color={0,0,127}));

  // Supply mass flow rate (to MassFlowSource_T) - THIS IS THE KEY FIX!
  connect(flowConvert.y, supply.m_flow_in)
    annotation (Line(points={{-159,48},{-150,48},{-150,80},{-140,80},{-140,72}},
                                                                       color={0,0,127}));

  // ============================================================================
  // BUILDING FLUID CONNECTIONS
  // ============================================================================

  // Supply → Building inlet
  connect(supply.ports[1], building.port_a)
    annotation (Line(points={{-120,64},{-50,64},{-50,68.4444},{-20,68.4444}},
                                                                  color={0,127,255}));

  // Building outlet → Return sensor → Sink
  connect(building.port_b, TReturn.port_a)
    annotation (Line(points={{-20,1.77778},{-74,1.77778},{-74,-60},{-80,-60}},
                                                                       color={0,127,255}));
  connect(TReturn.port_b, return_sink.ports[1])
    annotation (Line(points={{-100,-60},{-120,-60}}, color={0,127,255}));

  // ============================================================================
  // OUTPUT CONVERSIONS (K → °C)
  // ============================================================================

  // Zone temperatures
  T_roomIs_degC = building.TZone_living - 273.15;
  T_cellarIs_degC = building.TZone_cellar - 273.15;
  T_roofIs_degC = building.TZone_roof - 273.15;

  // Return water temperature
  STM_HCRL_Set_degC = TReturn.T - 273.15;  // 25 Dec 12.00 Already in degC, no conversion needed

  // Volume flow setpoint - pass through input value
  SFW_HCRLbM_Set_l_per_min = SFW_HCRLbM_l_per_min;

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
          textString="PHiL Building - Variable Internal Gains")}
        // ====== SKY BACKGROUND ======

        // ====== GROUND ======

        // ====== SUN ======

        // ====== HOUSE MAIN BODY ======

        // ====== ROOF ======

        // ====== CELLAR ======

        // ====== LIVING ZONE ======

        // ====== ROOF ZONE ======

        // ====== DOOR ======

        // ====== WINDOWS ======

        // ====== CHIMNEY ======

        // ====== INPUT LABELS - LEFT SIDE (PHiL + Occupancy) ======

        // ====== OUTPUT LABELS - RIGHT SIDE ======

        // ====== MODEL NAME ======

        // ====== SUBTITLE ======
),  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-240,-120},{240,180}})),
    experiment(
      StartTime=0,
      StopTime=259200,
      Interval=60,
      Tolerance=1e-06),
    Documentation(info="<html>
<h4>ThreeZoneBuilding_PHiL - Clean Version</h4>
<p>PHiL wrapper for three-zone building with organized connectors.</p>
<h5>Inputs (Left side):</h5>
<ul>
<li><b>T_supply</b> - Supply water temperature [degC]</li>
<li><b>Flow</b> - Volume flow rate [L/min]</li>
<li><b>T_ambient</b> - Outdoor temperature [degC]</li>
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
<li><b>Flow_set</b> - Flow setpoint [L/min]</li>
</ul>
</html>"));
end ThreeZoneBuilding_PHiL_basic;
