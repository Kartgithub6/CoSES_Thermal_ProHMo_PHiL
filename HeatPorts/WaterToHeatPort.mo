within CoSES_Thermal_ProHMo_PHiL.HeatPorts;
model WaterToHeatPort
  "Non-intrusive adapter between hydronic loop and HeatPort"

  // Use the same medium as your hydronic loop (radiator + boundaries)
  replaceable package Medium = IBPSA.Media.Water
    annotation(choicesAllMatching = true); // 25 Dec 09.00 Modelica.Media.Water.StandardWater
  // 10 Dec 22.00 IBPSA.Media.Water.StandardWater

  // 11 Dec 19.00 commented  outer Modelica.Fluid.System system;

  // Fluid ports (pass-through)
  Modelica.Fluid.Interfaces.FluidPort_a port_a(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));

  Modelica.Fluid.Interfaces.FluidPort_b port_b(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));

  // Thermal port (just a hub for radiator ↔ zone)
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heatPort
    annotation (Placement(transformation(extent={{-10,80},{10,100}})));
  // 11 Dec 19.00
protected
  // Mass flow from port_a into the component (standard IBPSA convention)
  Medium.MassFlowRate m_flow;

  // 14 Dec 19.00 — Add zone enthalpy here (DO NOT assign in equation section)
  Medium.SpecificEnthalpy h_zone;

  /* // 14 Dec 19.00
  // Specific enthalpies at ports
  Medium.SpecificEnthalpy h_a;
  Medium.SpecificEnthalpy h_b;
*/
equation
  // 15 Dec 11.00 added
  // Mass flow continuity
  m_flow = port_a.m_flow;
  port_a.m_flow + port_b.m_flow = 0;

  // Same pressure on both ports
  port_b.p = port_a.p;

  // 25 Dec 10.00 also interchanged the h_zone equation for sign convention CORRECT: Because we SET `port_b.h_outflow = h_zone`, the water actually leaves at zone temperature. The heat delivered should be:
  heatPort.Q_flow = m_flow * (h_zone - inStream(port_a.h_outflow));

  /* 25 Dec 10.00 
  // Heat flow from water to zone
  heatPort.Q_flow =
    m_flow * (inStream(port_a.h_outflow) - inStream(port_b.h_outflow));
*/
  // Water enthalpy at zone temperature (StandardWater compliant)
  h_zone =
    Medium.specificEnthalpy(
      Medium.setState_pT(port_a.p, heatPort.T));

  // Water leaves adapter at zone temperature
  port_a.h_outflow = h_zone;
  port_b.h_outflow = h_zone;

  /* // 15 Dec 11.00 commented
  m_flow = port_a.m_flow;
  port_b.p = port_a.p;
  port_a.m_flow + port_b.m_flow = 0;

  heatPort.Q_flow = m_flow * (inStream(port_a.h_outflow) - inStream(port_b.h_outflow));

  h_zone = Medium.specificEnthalpy(Medium.setState_pT(port_a.p, heatPort.T));


  port_a.h_outflow = h_zone;
  port_b.h_outflow = h_zone;

*/
  /* // 14 Dec 19.00
  // Enthalpy of water coming from inlet (supply side)
  h_a = inStream(port_a.h_outflow);
  // Energy balance: water enthalpy drop equals heat delivered to zone
  heatPort.Q_flow = m_flow * (h_a - port_b.h_outflow);
*/




  /* // 14 Dec 19.00 comented
  // Use only ONE inStream (avoid circular equalities)
  h_b = inStream(port_b.h_outflow);
  h_a = h_b; // assume balanced flow

  // Compute heat transfer
  heatPort.Q_flow = m_flow * (h_a - h_b);
*/
  /* // 14 Dec 18.00
  // Mass flow is taken from port_a (positive into component)
  m_flow = port_a.m_flow;

  // 14 Dec 14.00 added No pressure drop in adapter
  port_b.p = port_a.p;

  // Retrieve incoming enthalpies using inStream()
  h_a = inStream(port_a.h_outflow);
  h_b = inStream(port_b.h_outflow);

  // Heat transferred from water → zone
  heatPort.Q_flow = m_flow * (h_a - h_b);
  heatPort.T = heatPort.T; // 14 Dec 17.00 added

  // Enthalpy balance on fluid ports
  port_a.h_outflow = h_b;
  port_b.h_outflow = h_a;

  // No storage → zero net mass accumulation
  0 = port_a.m_flow + port_b.m_flow;

  /* // 14 Dec 11.00 commented
  // No heat capacity → water does not store energy
  heatPort.T = heatPort.T;
*/





  /* // 10 Dec 20.00
model WaterToHeatPort
  "Non-intrusive adapter between hydronic loop and HeatPort"

  replaceable package Medium = IBPSA.Media.Water
    annotation (choicesAllMatching=true);
  outer Modelica.Fluid.System system;

  // Fluid ports (pass-through)
  Modelica.Fluid.Interfaces.FluidPort_a port_a(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));

  Modelica.Fluid.Interfaces.FluidPort_b port_b(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));

  // Thermal port (to zone)
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b heatPort
    annotation (Placement(transformation(extent={{-10,80},{10,100}})));

protected 
  Modelica.Units.SI.MassFlowRate m_flow;

equation 
  // -------------------------
  // Fluid pass-through
  // -------------------------
  m_flow        = port_a.m_flow;
  port_b.m_flow = -port_a.m_flow;

  // No pressure drop in adapter
  //port_b.p = port_a.p;

  // Adapter does NOT change water enthalpy:
  // it simply passes through what comes from the other side.
  port_a.h_outflow = inStream(port_b.h_outflow);
  port_b.h_outflow = inStream(port_a.h_outflow);

  // -------------------------
  // Thermal behaviour
  // -------------------------
  // Heat taken from water equals enthalpy drop times mass flow.
  // (Positive Q_flow = heat delivered to the zone)
  heatPort.Q_flow =
      m_flow * (inStream(port_a.h_outflow) - inStream(port_b.h_outflow));

  // IMPORTANT: we do NOT set heatPort.T here.
  // The connected zone / thermal model determines the common temperature.
  heatPort.T = 293.15;

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
  Diagram(coordinateSystem(extent={{-100,-100},{100,100}})));
end WaterToHeatPort;
*/

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
  Diagram(coordinateSystem(extent={{-100,-100},{100,100}})));
end WaterToHeatPort;
