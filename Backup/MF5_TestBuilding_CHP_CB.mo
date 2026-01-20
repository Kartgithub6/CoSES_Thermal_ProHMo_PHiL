within CoSES_Thermal_ProHMo_PHiL.Backup;
model MF5_TestBuilding_CHP_CB
  "Test model integrating CHP + Condensing Boiler with ThreeZoneBuilding"

  // ============================================================================
  // DESCRIPTION
  // ============================================================================
  // This model demonstrates how to connect:
  // - SimpleCHP (primary heat source, also generates electricity)
  // - SimpleCondensingBoiler (backup/peak load heat source)
  // - ThreeZoneBuilding_PHiL (the building load)
  //
  // Control Strategy (similar to SimulationX MF5):
  // 1. CHP runs as base load (priority for electrical generation)
  // 2. Condensing Boiler provides backup when CHP can't meet demand
  // 3. Both are controlled based on living zone temperature
  // ============================================================================

  // ============================================================================
  // HEAT GENERATORS
  // ============================================================================

  Backup.SimpleCHP chp(
    QHeat_nominal=12500,
    // 12.5 kW thermal
    PElec_nominal=5000,
    // 5 kW electrical
    eta_heat_nominal=0.60,
    eta_elec_nominal=0.25,
    ModulationMin=0.30,
    tau=60) "Combined Heat and Power unit - primary heat source"
    annotation (Placement(transformation(extent={{-98,62},{-58,102}})));

  Backup.SimpleCondensingBoiler cb(
    QHeat_nominal=50000,
    // 50 kW thermal (backup)
    eta_nominal=0.98,
    eta_noncondensing=0.88,
    ModulationMin=0.20,
    tau=30) "Condensing Boiler - backup/peak heat source"
    annotation (Placement(transformation(extent={{-160,-20},{-120,20}})));

  // ============================================================================
  // THE BUILDING
  // ============================================================================

  ThermoEnergeticAnalysis.ThreeZoneBuilding_PHiL building(
    AZone_cellar=80,
    AZone_living=100,
    AZone_roof=60,
    hZone_cellar=2.2,
    hZone_living=2.5,
    hZone_roof=2.3,
    TZoneInit_cellar=288.15,
    TZoneInit_living=293.15,
    // Start at 20°C
    TZoneInit_roof=289.15,
    TRef_cellar=288.15,
    TRef_living=294.15,
    // Setpoint 21°C
    TRef_roof=294.15,
    TOutdoor=278.15,
    cellarHeat=true,
    roofHeat=true) "Three-zone building model"
    annotation (Placement(transformation(extent={{60,-50},{140,50}})));

  // ============================================================================
  // CONTROL PARAMETERS
  // ============================================================================

  parameter Modelica.Units.SI.Temperature T_CHP_on = 293.15
    "CHP turns ON below this temp [K] (20°C)"
    annotation(Dialog(group="CHP Control"));
  parameter Modelica.Units.SI.Temperature T_CHP_off = 295.15
    "CHP turns OFF above this temp [K] (22°C)"
    annotation(Dialog(group="CHP Control"));

  parameter Modelica.Units.SI.Temperature T_CB_on = 292.15
    "CB turns ON below this temp [K] (19°C) - more aggressive"
    annotation(Dialog(group="CB Control"));
  parameter Modelica.Units.SI.Temperature T_CB_off = 294.15
    "CB turns OFF above this temp [K] (21°C)"
    annotation(Dialog(group="CB Control"));

  parameter Real qv_nominal = 6
    "Nominal volume flow rate [L/min]"
    annotation(Dialog(group="Hydraulic"));

  // ============================================================================
  // ENVIRONMENT
  // ============================================================================

  Modelica.Blocks.Sources.Sine ambientTemp(
    amplitude = 3,
    f = 1/86400,
    offset = 5,
    phase = -1.5708)
    "Daily outdoor temperature variation [°C]: 2-8°C"
    annotation(Placement(transformation(extent={{-60,-80},{-40,-60}})));

  // ============================================================================
  // INTERNAL GAINS (Constant for simplicity - can be replaced with schedules)
  // ============================================================================

  Modelica.Blocks.Sources.Constant nPersons_living(k=2)
    annotation(Placement(transformation(extent={{-20,38},{0,58}})));
  Modelica.Blocks.Sources.Constant nPersons_cellar(k=0)
    annotation(Placement(transformation(extent={{-22,-4},{-2,16}})));
  Modelica.Blocks.Sources.Constant nPersons_roof(k=0)
    annotation(Placement(transformation(extent={{-20,-44},{0,-24}})));

  Modelica.Blocks.Sources.Constant P_app_living(k=200)
    annotation(Placement(transformation(extent={{180,20},{160,40}})));
  Modelica.Blocks.Sources.Constant P_app_cellar(k=80)
    annotation(Placement(transformation(extent={{180,-10},{160,10}})));
  Modelica.Blocks.Sources.Constant P_app_roof(k=50)
    annotation(Placement(transformation(extent={{180,-40},{160,-20}})));

  // ============================================================================
  // FLOW RATE SOURCE
  // ============================================================================

  Modelica.Blocks.Sources.Constant flowRate(k=qv_nominal)
    "Circulation pump flow rate [L/min]"
    annotation(Placement(transformation(extent={{-220,30},{-200,50}})));

  // ============================================================================
  // CONTROL SIGNALS
  // ============================================================================

  // CHP hysteresis control
  Modelica.Blocks.Logical.Hysteresis CHP_hysteresis(
    uLow = T_CHP_on,
    uHigh = T_CHP_off,
    pre_y_start = true)
    "CHP on when T_living < T_CHP_on, off when T_living > T_CHP_off"
    annotation(Placement(transformation(extent={{-220,130},{-200,150}})));

  Modelica.Blocks.Logical.Not CHP_on_signal
    "Invert hysteresis (ON when temp is LOW)"
    annotation(Placement(transformation(extent={{-190,130},{-170,150}})));

  // CB hysteresis control (more aggressive - backup)
  Modelica.Blocks.Logical.Hysteresis CB_hysteresis(
    uLow = T_CB_on,
    uHigh = T_CB_off,
    pre_y_start = false)
    "CB on when T_living < T_CB_on, off when T_living > T_CB_off"
    annotation(Placement(transformation(extent={{-220,90},{-200,110}})));

  Modelica.Blocks.Logical.Not CB_on_signal
    "Invert hysteresis"
    annotation(Placement(transformation(extent={{-190,90},{-170,110}})));

  // Modulation based on temperature error
  Modelica.Blocks.Math.Add tempError(k1=-1, k2=1)
    "Temperature error = Setpoint - Actual"
    annotation(Placement(transformation(extent={{-220,60},{-200,80}})));

  Modelica.Blocks.Sources.Constant T_setpoint_K(k=294.15)
    "Living zone setpoint [K] (21°C)"
    annotation(Placement(transformation(extent={{-260,50},{-240,70}})));

  Modelica.Blocks.Math.Gain modulationGain(k=0.2)
    "Convert temp error to modulation (0.2 = 5K error gives 100%)"
    annotation(Placement(transformation(extent={{-190,60},{-170,80}})));

  Modelica.Blocks.Nonlinear.Limiter modulationLimiter(
    uMax=1,
    uMin=0)
    "Limit modulation to 0-1 range"
    annotation(Placement(transformation(extent={{-160,60},{-140,80}})));

  // ============================================================================
  // TEMPERATURE MIXING (CHP + CB outputs combined)
  // ============================================================================

  // When both are running, we need to mix their outputs
  // Simplified approach: weighted average based on heat output

protected
  Modelica.Units.SI.Temperature T_supply_mixed_K "Mixed supply temperature [K]";
  Real Q_total_kW "Total heat output [kW]";

public
  // Output for plotting
  Real T_supply_mixed_degC = T_supply_mixed_K - 273.15 "Mixed supply temp [°C]";
  Real T_return_degC = building.STM_HCRL_Set_degC "Return temperature [°C]";
  Real T_living_degC = building.T_roomIs_degC "Living zone temp [°C]";
  Real T_cellar_degC = building.T_cellarIs_degC "Cellar temp [°C]";
  Real T_roof_degC = building.T_roofIs_degC "Roof temp [°C]";

  Real CHP_QHeat_kW = chp.QHeat_kW "CHP heat output [kW]";
  Real CHP_PElec_kW = chp.PElec_kW "CHP electrical output [kW]";
  Real CB_QHeat_kW = cb.QHeat_kW "CB heat output [kW]";
  Real Total_QHeat_kW = Q_total_kW "Total heat output [kW]";

  Real time_hours = time/3600 "Simulation time [hours]";
  Real time_days = time/86400 "Simulation time [days]";

equation
  // ============================================================================
  // CONTROL CONNECTIONS
  // ============================================================================

  // Living zone temperature for control (convert °C to K)
  CHP_hysteresis.u = building.T_roomIs_degC + 273.15;
  CB_hysteresis.u = building.T_roomIs_degC + 273.15;
  tempError.u2 = building.T_roomIs_degC + 273.15;

  connect(T_setpoint_K.y, tempError.u1);

  connect(CHP_hysteresis.y, CHP_on_signal.u);
  connect(CB_hysteresis.y, CB_on_signal.u);

  connect(tempError.y, modulationGain.u);
  connect(modulationGain.y, modulationLimiter.u);

  // ============================================================================
  // HEAT GENERATOR CONNECTIONS
  // ============================================================================

  // CHP inputs
  connect(CHP_on_signal.y, chp.CHPon);
  connect(modulationLimiter.y, chp.Modulation);
  chp.TReturn_degC = building.STM_HCRL_Set_degC;  // Return from building
  connect(flowRate.y, chp.qv_l_per_min);

  // CB inputs
  connect(CB_on_signal.y, cb.CBon);
  connect(modulationLimiter.y, cb.Modulation);
  cb.TReturn_degC = building.STM_HCRL_Set_degC;  // Return from building
  connect(flowRate.y, cb.qv_l_per_min);

  // ============================================================================
  // SUPPLY TEMPERATURE MIXING
  // ============================================================================

  Q_total_kW = chp.QHeat_kW + cb.QHeat_kW;

  // Weighted average of supply temperatures based on heat output
  if Q_total_kW > 0.1 then
    T_supply_mixed_K = (chp.QHeat_kW * (chp.TSupply_degC + 273.15) +
                        cb.QHeat_kW * (cb.TSupply_degC + 273.15)) / Q_total_kW;
  else
    // No heat output: supply = return (or ambient)
    T_supply_mixed_K = building.STM_HCRL_Set_degC + 273.15;
  end if;

  // ============================================================================
  // BUILDING CONNECTIONS
  // ============================================================================

  // Supply temperature to building
  building.STM_HCVLaM_degC = T_supply_mixed_degC;

  // Flow rate to building
  connect(flowRate.y, building.SFW_HCRLbM_l_per_min);

  // Ambient temperature
  connect(ambientTemp.y, building.T_ambient_degC);

  // Internal gains
  connect(nPersons_living.y, building.nPersons_living_in);
  connect(nPersons_cellar.y, building.nPersons_cellar_in);
  connect(nPersons_roof.y, building.nPersons_roof_in);
  connect(P_app_living.y, building.P_appliances_living_W_in);
  connect(P_app_cellar.y, building.P_appliances_cellar_W_in);
  connect(P_app_roof.y, building.P_appliances_roof_W_in);

  annotation(
    experiment(
      StartTime=0,
      StopTime=259200,    // 3 days
      Interval=60,
      Tolerance=1e-06),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-280,-100},{200,180}})),
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-280,-100},{200,180}}),
      graphics={
        Rectangle(extent={{-280,180},{200,-100}}, lineColor={0,0,0},
          fillColor={255,255,255}, fillPattern=FillPattern.Solid),
        Rectangle(extent={{-240,140},{-140,60}}, lineColor={255,128,0},
          fillColor={255,200,150}, fillPattern=FillPattern.Solid),
        Text(extent={{-230,120},{-150,80}}, textColor={255,128,0},
          textString="CHP", textStyle={TextStyle.Bold}),
        Rectangle(extent={{-240,40},{-140,-40}}, lineColor={0,0,255},
          fillColor={200,200,255}, fillPattern=FillPattern.Solid),
        Text(extent={{-230,20},{-150,-20}}, textColor={0,0,200},
          textString="CB", textStyle={TextStyle.Bold}),
        Rectangle(extent={{0,100},{160,-60}}, lineColor={139,69,19},
          fillColor={255,228,196}, fillPattern=FillPattern.Solid),
        Polygon(points={{0,100},{80,160},{160,100},{0,100}}, lineColor={178,34,34},
          fillColor={178,34,34}, fillPattern=FillPattern.Solid),
        Text(extent={{20,60},{140,20}}, textColor={0,0,0},
          textString="Building"),
        Line(points={{-140,100},{0,50}}, color={255,0,0}, thickness=1),
        Line(points={{-140,0},{0,0},{0,50}}, color={255,0,0}, thickness=1),
        Line(points={{0,-30},{-140,-30},{-140,60}}, color={0,0,255}, thickness=1),
        Text(extent={{-200,175},{120,155}}, textColor={0,0,255},
          textString="CHP + CB + Building Test", textStyle={TextStyle.Bold})}),
    Documentation(info="<html>
<h4>TestBuilding_CHP_CB</h4>
<p>Integrated test model demonstrating CHP + Condensing Boiler heating system.</p>

<h5>System Components:</h5>
<ul>
<li><b>CHP</b>: 12.5 kW thermal, 5 kW electrical (primary heat source)</li>
<li><b>CB</b>: 50 kW thermal (backup/peak load)</li>
<li><b>Building</b>: Three-zone building with hydronic heating</li>
</ul>

<h5>Control Strategy:</h5>
<ul>
<li>CHP runs when living temp &lt; 20°C, stops when &gt; 22°C</li>
<li>CB runs when living temp &lt; 19°C, stops when &gt; 21°C (backup)</li>
<li>Both modulate based on temperature error</li>
</ul>

<h5>Variables to Plot:</h5>
<ul>
<li>Temperatures: T_living_degC, T_cellar_degC, T_roof_degC</li>
<li>Heat output: CHP_QHeat_kW, CB_QHeat_kW, Total_QHeat_kW</li>
<li>Electrical: CHP_PElec_kW</li>
<li>Supply/Return: T_supply_mixed_degC, T_return_degC</li>
</ul>

<h5>Expected Behavior:</h5>
<ol>
<li>Building starts at 20°C, needs heating</li>
<li>CHP turns on, provides 12.5 kW</li>
<li>If temp drops below 19°C, CB also turns on</li>
<li>As building heats up, both modulate down</li>
<li>CHP turns off at 22°C, CB at 21°C</li>
</ol>
</html>"));
end MF5_TestBuilding_CHP_CB;
