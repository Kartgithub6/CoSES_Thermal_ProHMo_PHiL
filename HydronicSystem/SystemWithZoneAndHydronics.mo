within CoSES_Thermal_ProHMo_PHiL.HydronicSystem;
model SystemWithZoneAndHydronics
  "Hydronic loop segment + thermal port for zone connection"

  /* // 10 Dec 18.00
  replaceable package Medium = IBPSA.Media.Water
    annotation(choicesAllMatching=true);
  */
  replaceable package Medium = IBPSA.Media.Water
    annotation(choicesAllMatching=true); // 25 Dec 09.00 Modelica.Media.Water.StandardWater

  // 10 Dec 22.00
  outer Modelica.Fluid.System system;

  // Fluid in/out
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));

  // Thermal output to zone
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_zone
    annotation (Placement(transformation(extent={{-10,80},{10,100}})));


  // Adapter between hydronics and heat port
/*
  // Simple radiator
  IBPSA.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 radiator(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
  */
  // 13 Dec 18.00 added -  Thermal collector to combine convective and radiative heat from radiator

  /* // 13 Dec 18.00 commented
  HeatPorts.WaterToHeatPort waterToHeatPort(
    redeclare package Medium = Medium)
  annotation (Placement(transformation(extent={{-10,4},{10,24}})));
  */
  HeatPorts.WaterToHeatPort waterToHeatPort(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-54,28},{-34,48}})));
equation
  // 1. Hydronic through flow

  // 2. Thermal output of radiator goes into adapter (heat extracted from water)

  // 3. Send heat from adapter to outside zone

  // 4. Hydronic nodes to adapter (pass-through)














/*  
model SystemWithZoneAndHydronics
  "Simple hydronic interface: boiler loop -> radiator -> zone heatPort"

  replaceable package Medium = Modelica.Media.Water.StandardWater
    annotation (choicesAllMatching = true);

  // --- Fluid side towards boiler / main PHiL loop ---
  Modelica.Fluid.Interfaces.FluidPort_a port_a(
    redeclare package Medium = Medium)
    "Supply from boiler / CHP loop";

  Modelica.Fluid.Interfaces.FluidPort_b port_b(
    redeclare package Medium = Medium)
    "Return to boiler / CHP loop";

  // --- Thermal side towards building zone ---
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b zoneHeatPort
    "Connect this to HeatedZone_1.heatPort";

  // --- Parameters ---
  parameter Real cpWater = 4180
    "Specific heat capacity of water [J/(kg.K)]";

  parameter Modelica.Units.SI.ThermalConductance G_rad = 2000
    "Overall UA of radiator / heat exchanger";

protected 
  // Adapter: converts water ΔT, m_flow -> Q_flow into thermal port
  WaterToHeatPort adapter(
    redeclare package Medium = Medium,
    cp = cpWater);

  // Simple radiator / HEX between adapter and zone
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor radiator(
    G = G_rad);

equation 
  // Fluid connections
  connect(port_a, adapter.port_a);
  connect(port_b, adapter.port_b);

  // Thermal side: water → radiator → zone
  connect(adapter.heatPort, radiator.port_a);
  connect(radiator.port_b, zoneHeatPort);


//old
model SystemWithZoneAndHydronics
  HeatedZone_1 heatedZone_1
    annotation (Placement(transformation(extent={{-4,20},{16,40}})));
  HeatPorts.WaterToHeatPort waterToHeatPort
    annotation (Placement(transformation(extent={{36,14},{56,34}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SystemWithZoneAndHydronics;
  
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)));
end SystemWithZoneAndHydronics;
*/









  // 13 Dec 18.00 added
  /* // 13 Dec 18.00 commented
  connect(radiator.heatPortCon, waterToHeatPort.heatPort) annotation (Line(
        points={{-2,-22.8},{-2,0},{-14,0},{-14,28},{0,28},{0,23}}, color={191,0,
          0}));
  connect(waterToHeatPort.heatPort, zoneHeatPort)
  annotation (Line(points={{0,23},{0,90}}, color={191,0,0}));
  
  connect(radiator.heatPortRad, waterToHeatPort.heatPort) annotation (Line(
        points={{2,-22.8},{2,0},{16,0},{16,28},{0,28},{0,23}}, color={191,0,0}));
annotation(
  Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
  Icon(graphics={Rectangle(extent={{-80,-60},{80,60}}, fillPattern=FillPattern.Solid, fillColor={240,240,240}),
  Text(extent={{-70,20},{70,-20}},textString="Hydronics+Zone")}));
  */
  /* // 13 Dec 17.00 commented
  connect(port_a, waterToHeatPort.port_a)
  annotation (Line(points={{-90,0},{-90,14},{-9,14}}, color={0,127,255}));
  connect(waterToHeatPort.port_b, port_b)
  annotation (Line(points={{9,14},{90,14},{90,0}}, color={0,127,255}));
  */

  connect(waterToHeatPort.port_a, port_a)
    annotation (Line(points={{-53,38},{-88,38},{-88,0}}, color={0,127,255}));
  connect(port_zone, waterToHeatPort.heatPort) annotation (Line(points={{0,90},
          {0,54},{-44,54},{-44,47}}, color={191,0,0}));
  connect(port_b, waterToHeatPort.port_b)
    annotation (Line(points={{90,0},{90,38},{-35,38}}, color={0,127,255}));
end SystemWithZoneAndHydronics;
