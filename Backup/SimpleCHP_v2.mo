within CoSES_Thermal_ProHMo_PHiL.Backup;
model SimpleCHP_v2
  "Simplified CHP model v2 - with debug outputs and verified physics"

  // ============================================================================
  // PARAMETERS
  // ============================================================================

  parameter Modelica.Units.SI.Power QHeat_nominal = 12500
    "Nominal thermal power output [W]"
    annotation(Dialog(group="Power Rating"));

  parameter Modelica.Units.SI.Power PElec_nominal = 5000
    "Nominal electrical power output [W]"
    annotation(Dialog(group="Power Rating"));

  parameter Real eta_heat = 0.60 "Thermal efficiency [-]"
    annotation(Dialog(group="Efficiency"));

  parameter Real eta_elec = 0.25 "Electrical efficiency [-]"
    annotation(Dialog(group="Efficiency"));

  parameter Real ModulationMin = 0.30 "Minimum modulation (30%)"
    annotation(Dialog(group="Control"));

  parameter Modelica.Units.SI.Temperature TFlow_max = 363.15
    "Maximum flow temperature [K] (90°C)"
    annotation(Dialog(group="Limits"));

  parameter Modelica.Units.SI.Time tau = 60 "Time constant [s]"
    annotation(Dialog(group="Dynamics"));

  // Fixed medium properties
  parameter Real cp = 4180 "Specific heat [J/(kg·K)]";
  parameter Real rho = 1000 "Density [kg/m³]";

  // ============================================================================
  // INPUTS
  // ============================================================================

  Modelica.Blocks.Interfaces.BooleanInput CHPon "CHP on/off"
    annotation(Placement(transformation(extent={{-140,60},{-100,100}})));

  Modelica.Blocks.Interfaces.RealInput Modulation "Modulation [0-1]"
    annotation(Placement(transformation(extent={{-140,10},{-100,50}})));

  Modelica.Blocks.Interfaces.RealInput TReturn_degC "Return temp [°C]"
    annotation(Placement(transformation(extent={{-140,-50},{-100,-10}})));

  Modelica.Blocks.Interfaces.RealInput qv_l_per_min "Flow [L/min]"
    annotation(Placement(transformation(extent={{-140,-100},{-100,-60}})));

  // ============================================================================
  // OUTPUTS
  // ============================================================================

  Modelica.Blocks.Interfaces.RealOutput TSupply_degC "Supply temp [°C]"
    annotation(Placement(transformation(extent={{100,60},{120,80}})));

  Modelica.Blocks.Interfaces.RealOutput QHeat_kW "Heat output [kW]"
    annotation(Placement(transformation(extent={{100,20},{120,40}})));

  Modelica.Blocks.Interfaces.RealOutput PElec_kW "Electrical output [kW]"
    annotation(Placement(transformation(extent={{100,-20},{120,0}})));

  // ============================================================================
  // DEBUG OUTPUTS - Check these in simulation!
  // ============================================================================

  Real debug_m_flow_kg_s "Mass flow [kg/s] - should be ~0.1 for 6 L/min";
  Real debug_deltaT_K "Temperature rise [K] - should be ~30K at full power";
  Real debug_ModActual "Actual modulation - should be 0.3-1.0 when ON";
  Real debug_QHeat_W "Heat power [W] - should be up to 12500W";

  // ============================================================================
  // STATE VARIABLES
  // ============================================================================

protected
  Modelica.Units.SI.Power QHeat_actual(start=0, fixed=true) "Actual heat [W]";

equation
  // ============================================================================
  // FLOW CONVERSION
  // ============================================================================
  debug_m_flow_kg_s = rho * (qv_l_per_min / 60000);  // L/min → m³/s → kg/s

  // ============================================================================
  // MODULATION
  // ============================================================================
  debug_ModActual = if CHPon then max(ModulationMin, min(1.0, Modulation)) else 0;

  // ============================================================================
  // HEAT DYNAMICS
  // ============================================================================
  tau * der(QHeat_actual) = debug_ModActual * QHeat_nominal - QHeat_actual;

  debug_QHeat_W = QHeat_actual;

  // ============================================================================
  // TEMPERATURE CALCULATION
  // ============================================================================
  // deltaT = Q / (m_flow * cp)

  if debug_m_flow_kg_s > 0.001 then
    debug_deltaT_K = QHeat_actual / (debug_m_flow_kg_s * cp);
  else
    debug_deltaT_K = 0;
  end if;

  TSupply_degC = min(TFlow_max - 273.15, TReturn_degC + debug_deltaT_K);

  // ============================================================================
  // OUTPUTS
  // ============================================================================
  QHeat_kW = QHeat_actual / 1000;
  PElec_kW = (QHeat_actual / QHeat_nominal) * PElec_nominal / 1000;

  annotation(
    Icon(graphics={
      Rectangle(extent={{-100,100},{100,-100}}, fillColor={255,200,150},
        fillPattern=FillPattern.Solid, lineColor={255,128,0}),
      Text(extent={{-80,40},{80,-40}}, textString="CHP v2",
        textColor={255,128,0}, textStyle={TextStyle.Bold}),
      Text(extent={{-100,100},{100,80}}, textString="%name", textColor={0,0,255})}),
    Documentation(info="<html>
<h4>SimpleCHP_v2 - Debuggable Version</h4>
<p>Check these debug outputs:</p>
<ul>
<li><b>debug_m_flow_kg_s</b>: Should be ~0.1 for 6 L/min</li>
<li><b>debug_ModActual</b>: Should be 0.3-1.0 when ON</li>
<li><b>debug_QHeat_W</b>: Should be up to 12500 W</li>
<li><b>debug_deltaT_K</b>: Should be ~30K at full power with 6 L/min</li>
</ul>
</html>"));
end SimpleCHP_v2;
