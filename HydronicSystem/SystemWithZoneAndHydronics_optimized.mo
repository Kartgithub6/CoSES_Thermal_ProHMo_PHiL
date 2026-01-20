within CoSES_Thermal_ProHMo_PHiL.HydronicSystem;
model SystemWithZoneAndHydronics_optimized
  "Hydronic loop with EN442-2 radiator - FULLY INITIALIZED"

  replaceable package Medium = Modelica.Media.Water.StandardWater
    annotation(choicesAllMatching=true);
  outer Modelica.Fluid.System system;

  // ============================================================================
  // CONNECTORS
  // ============================================================================

  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    "Hot water inlet from valve"
    annotation(Placement(visible=true,transformation(origin={-100,0},extent={{-10,-10},{10,10}},rotation=0),
    iconTransformation(origin={-100,0},extent={{-10,-10},{10,10}},rotation=0)));

  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    "Return water outlet"
    annotation(Placement(visible=true,transformation(origin={100,0},extent={{-10,-10},{10,10}},rotation=0),
    iconTransformation(origin={100,0},extent={{-10,-10},{10,10}},rotation=0)));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_zone
    "Combined heat port connection to building zone"
    annotation(Placement(visible=true,transformation(origin={0,100},extent={{-10,-10},{10,10}},rotation=0),
    iconTransformation(origin={0,100},extent={{-10,-10},{10,10}},rotation=0)));

  // ============================================================================
  // RADIATOR PARAMETERS (EN442-2 Standard)
  // ============================================================================

  parameter Modelica.Units.SI.Power Q_flow_nominal=1500
    "Nominal heating power at design conditions [W]"
    annotation (Dialog(group="Radiator Design"));

  parameter Modelica.Units.SI.Temperature T_a_nominal=343.15
    "Nominal water inlet temperature (70°C)"
    annotation (Dialog(group="Radiator Design"));

  parameter Modelica.Units.SI.Temperature T_b_nominal=323.15
    "Nominal water outlet temperature (50°C)"
    annotation (Dialog(group="Radiator Design"));

  parameter Modelica.Units.SI.Temperature TAir_nominal=293.15
    "Nominal room air temperature (20°C)"
    annotation (Dialog(group="Radiator Design"));

  parameter Real fraRad=0.35
    "Fraction of radiative heat transfer (typically 0.30-0.40)"
    annotation (Dialog(group="Radiator Design"));

  parameter Integer nEle=5
    "Number of radiator elements for discretization"
    annotation (Dialog(group="Advanced"));

  // ⭐ NEW: Medium initialization parameters
  parameter Modelica.Units.SI.Temperature T_start=323.15
    "Start temperature for radiator medium (50°C)"
    annotation (Dialog(group="Initialization"));

  parameter Modelica.Units.SI.Pressure p_start=200000
    "Start pressure for radiator medium (2 bar)"
    annotation (Dialog(group="Initialization"));

  // ============================================================================
  // COMPONENTS
  // ============================================================================

  // EN442-2 Standard Radiator from IBPSA
  // ⭐ WITH PROPER INITIALIZATION
  IBPSA.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 radiator(
    redeclare package Medium=Medium,
    Q_flow_nominal=Q_flow_nominal,
    T_a_nominal=T_a_nominal,
    T_b_nominal=T_b_nominal,
    TAir_nominal=TAir_nominal,
    fraRad=fraRad,
    nEle=nEle,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    T_start=T_start,
    p_start=p_start)
    "EN442-2 compliant radiator - FULLY INITIALIZED"
    annotation(Placement(visible=true,transformation(origin={0,-20},extent={{-10,-10},{10,10}},rotation=0)));

  // Thermal collector to combine convective and radiative heat
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(m=2)
    "Combines convective and radiative heat into single zone connection"
    annotation(Placement(visible=true,transformation(origin={0,60},extent={{-10,-10},{10,10}},rotation=90)));

equation
  // ============================================================================
  // FLUID CONNECTIONS
  // ============================================================================
  connect(port_a,radiator.port_a)
    annotation(Line(points={{-100,0},{-10,0},{-10,-20}},color={0,127,255}));
  connect(radiator.port_b,port_b)
    annotation(Line(points={{10,-20},{10,0},{100,0}},color={0,127,255}));

  // ============================================================================
  // THERMAL CONNECTIONS
  // ============================================================================
  connect(radiator.heatPortCon,thermalCollector.port_a[1])
    annotation(Line(points={{-2,-12},{-2,60},{-9.75,60}},color={191,0,0}));
  connect(radiator.heatPortRad,thermalCollector.port_a[2])
    annotation(Line(points={{2,-12},{2,60},{-10.25,60}},color={191,0,0}));
  connect(thermalCollector.port_b,port_zone)
    annotation(Line(points={{10,60},{10,80},{0,80},{0,100}},color={191,0,0}));

  annotation(
    Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(extent={{-100,100},{100,-100}},lineColor={0,0,0},fillColor={255,255,255},fillPattern=FillPattern.Solid),
        Rectangle(extent={{-80,60},{80,-60}},lineColor={0,0,0},fillColor={200,200,200},fillPattern=FillPattern.Solid),
        Line(points={{-100,0},{-80,0}},color={0,127,255},thickness=1),
        Line(points={{80,0},{100,0}},color={0,127,255},thickness=1),
        Line(points={{0,60},{0,100}},color={191,0,0},thickness=1),
        Text(extent={{-60,40},{60,20}},textString="RadiatorEN442_2",lineColor={0,0,0}),
        Text(extent={{-60,80},{60,60}},textString="%name",lineColor={0,0,255})}),
    Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,100}})),
    Documentation(info="<html>
<h4>SystemWithZoneAndHydronics_new_FINAL - PRODUCTION VERSION</h4>

<p>This is the FINAL corrected version with complete initialization to prevent simulation failures.</p>

<h5>Fixes Applied:</h5>
<ul>
<li><b>energyDynamics = SteadyStateInitial:</b> Prevents over-constrained initialization</li>
<li><b>massDynamics = SteadyStateInitial:</b> Consistent with energy dynamics</li>
<li><b>T_start = 323.15 K (50°C):</b> Explicit temperature start value</li>
<li><b>p_start = 200000 Pa (2 bar):</b> Explicit pressure start value</li>
</ul>

<p>These explicit start values are propagated to all 5 radiator elements, eliminating the
'using zero start values' warnings during translation.</p>

<h5>Parameters:</h5>
<ul>
<li><b>Q_flow_nominal:</b> Nominal heating power (default 1500W)</li>
<li><b>T_a_nominal:</b> Inlet temp (default 70°C)</li>
<li><b>T_b_nominal:</b> Outlet temp (default 50°C)</li>
<li><b>TAir_nominal:</b> Room temp (default 20°C)</li>
<li><b>fraRad:</b> Radiative fraction (default 0.35)</li>
<li><b>nEle:</b> Elements (default 5)</li>
<li><b>T_start:</b> Initial temperature (default 50°C) ⭐ NEW</li>
<li><b>p_start:</b> Initial pressure (default 2 bar) ⭐ NEW</li>
</ul>
</html>"));
end SystemWithZoneAndHydronics_optimized;
