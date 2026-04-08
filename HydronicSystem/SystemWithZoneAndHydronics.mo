within CoSES_Thermal_ProHMo_PHiL.HydronicSystem;
model SystemWithZoneAndHydronics
  "Hydronic loop segment + thermal port for zone connection"


  replaceable package Medium = IBPSA.Media.Water
    annotation(choicesAllMatching=true);

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

  HeatPorts.WaterToHeatPort waterToHeatPort(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-54,28},{-34,48}})));
equation
  connect(waterToHeatPort.port_a, port_a)
    annotation (Line(points={{-53,38},{-88,38},{-88,0}}, color={0,127,255}));
  connect(port_zone, waterToHeatPort.heatPort) annotation (Line(points={{0,90},
          {0,54},{-44,54},{-44,47}}, color={191,0,0}));
  connect(port_b, waterToHeatPort.port_b)
    annotation (Line(points={{90,0},{90,38},{-35,38}}, color={0,127,255}),
    Documentation(info="<html>
<h4>Hydronic Zone System</h4>
<p><b>Zone with Hydronic Systems</b></p>
<li>
Oct 3, 2025, by Karthik Murugesan
</li>
<p><b>Added components for Hydronic system</b></p>
</html>"));
end SystemWithZoneAndHydronics;
