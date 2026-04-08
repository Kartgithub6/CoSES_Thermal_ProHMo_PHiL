within CoSES_Thermal_ProHMo_PHiL.HeatPorts;
model WaterToHeatPort
  "Non-intrusive adapter between hydronic loop and HeatPort"

  // Use the same medium as your hydronic loop (radiator + boundaries)
  replaceable package Medium = IBPSA.Media.Water
    annotation(choicesAllMatching = true); // 25 Dec 09.00 Modelica.Media.Water.StandardWater
  // 10 Dec 22.00 IBPSA.Media.Water.StandardWater


  // Fluid ports (passin-through)
  Modelica.Fluid.Interfaces.FluidPort_a port_a(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));

  Modelica.Fluid.Interfaces.FluidPort_b port_b(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));

  // Thermal port (just a hub for radiator ↔ zone)
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heatPort
    annotation (Placement(transformation(extent={{-10,80},{10,100}})));

protected
  // Mass flow from port_a into the component (standard IBPSA convention)
  Medium.MassFlowRate m_flow;

  // 14 Dec 19.00 — Add zone enthalpy here (DO NOT assign in equation section)
  Medium.SpecificEnthalpy h_zone;

equation
  // Mass flow continuity
  m_flow = port_a.m_flow;
  port_a.m_flow + port_b.m_flow = 0;

  // Same pressure on both ports
  port_b.p = port_a.p;

  // 25 Dec 10.00 also interchanged the h_zone equation for sign convention CORRECT: Because we SET `port_b.h_outflow = h_zone`, the water actually leaves at zone temperature. The heat delivered should be:
  heatPort.Q_flow = m_flow * (h_zone - inStream(port_a.h_outflow));


  // Water enthalpy at zone temperature (StandardWater compliant)
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
           extent={{-50,10},{50,-10}},
           textString="Water↔Heat",
           lineColor={0,0,255})}),
  Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
    Documentation(info="<html>
<h4>Water to Heat Port</h4>
<li>
Oct 17, 2025, by Karthik Murugesan
</li>
<p><b>Ports with inlet and thermal connectors<b></p>
</html>"));

end WaterToHeatPort;
