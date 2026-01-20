within CoSES_Thermal_ProHMo_PHiL.HeatPorts;
model WaterToHeatPort_SimpleWater
  "Non-intrusive adapter between hydronic loop and HeatPort - SimpleWater version"

  // ============================================================================
  // CHANGE: Use ConstantPropertyLiquidWater instead of StandardWater
  // This avoids temperature limit errors (IF97 requires T >= 273.15 K)
  // ============================================================================

  replaceable package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater
    constrainedby Modelica.Media.Interfaces.PartialMedium
    annotation(choicesAllMatching = true);

  // Fluid ports (pass-through)
  Modelica.Fluid.Interfaces.FluidPort_a port_a(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));

  Modelica.Fluid.Interfaces.FluidPort_b port_b(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));

  // Thermal port (connects to zone)
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heatPort
    annotation (Placement(transformation(extent={{-10,80},{10,100}})));

protected
  // Mass flow from port_a into the component
  Medium.MassFlowRate m_flow;

  // Zone enthalpy
  Medium.SpecificEnthalpy h_zone;

equation
  // Mass flow continuity
  m_flow = port_a.m_flow;
  port_a.m_flow + port_b.m_flow = 0;

  // Same pressure on both ports (no pressure drop)
  port_b.p = port_a.p;

  // Heat flow from water to zone
  // Q = m_flow * (h_in - h_out)
  heatPort.Q_flow =
    m_flow * (inStream(port_a.h_outflow) - inStream(port_b.h_outflow));

  // Water enthalpy at zone temperature
  // NOTE: With ConstantPropertyLiquidWater, this works at any temperature
  h_zone =
    Medium.specificEnthalpy(
      Medium.setState_pT(port_a.p, heatPort.T));

  // Water leaves adapter at zone temperature
  port_a.h_outflow = h_zone;
  port_b.h_outflow = h_zone;

annotation(
  Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
       graphics={
         Rectangle(
           extent={{-60,-40},{60,40}},
           lineColor={0,0,255},
           fillPattern=FillPattern.Solid,
           fillColor={200,200,255}),
         Text(
           extent={{-50,20},{50,0}},
           textString="Water↔Heat",
           textColor={0,0,255}),
         Text(
           extent={{-50,-5},{50,-25}},
           textString="(SimpleWater)",
           textColor={0,128,0})}),
  Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
  Documentation(info="<html>
<h4>WaterToHeatPort_SimpleWater</h4>
<p>Same as WaterToHeatPort but uses ConstantPropertyLiquidWater medium.</p>
<p>This avoids IF97 temperature limit errors that occur when zone temperature
drops below 273.15 K during transients.</p>
</html>"));
end WaterToHeatPort_SimpleWater;
