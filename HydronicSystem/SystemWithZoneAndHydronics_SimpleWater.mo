within CoSES_Thermal_ProHMo_PHiL.HydronicSystem;
model SystemWithZoneAndHydronics_SimpleWater
  "Hydronic loop segment + thermal port for zone connection - SimpleWater version"

  // ============================================================================
  // CHANGE: Use ConstantPropertyLiquidWater instead of StandardWater
  // This avoids temperature limit errors (IF97 requires T >= 273.15 K)
  // ============================================================================

  replaceable package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater
    constrainedby Modelica.Media.Interfaces.PartialMedium
    annotation(choicesAllMatching=true);

  outer Modelica.Fluid.System system;

  // Fluid in/out
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium = Medium)
    "Supply water inlet"
    annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));

  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium = Medium)
    "Return water outlet"
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));

  // Thermal output to zone
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_zone
    "Heat port to building zone"
    annotation (Placement(transformation(extent={{-10,80},{10,100}})));

  // ============================================================================
  // INTERNAL: Water-to-HeatPort adapter (also using simple water)
  // ============================================================================

  // NOTE: This references the SimpleWater version of WaterToHeatPort
  // Make sure WaterToHeatPort_SimpleWater is in your package, or
  // change this to use the inline version below.

  HeatPorts.WaterToHeatPort_SimpleWater waterToHeatPort_SimpleWater(redeclare
      package Medium = Medium)
    annotation (Placement(transformation(extent={{-28,8},{20,56}})));
equation
  // Connect fluid ports through the adapter

  // Connect heat port to zone

  connect(port_a, waterToHeatPort_SimpleWater.port_a)
    annotation (Line(points={{-88,0},{-88,32},{-25.6,32}}, color={0,127,255}));
  connect(waterToHeatPort_SimpleWater.port_b, port_b)
    annotation (Line(points={{17.6,32},{90,32},{90,0}}, color={0,127,255}));
  connect(waterToHeatPort_SimpleWater.heatPort, port_zone) annotation (Line(
        points={{-4,53.6},{-4,76},{0,76},{0,90}}, color={191,0,0}));
annotation(
  Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
       graphics={
         Rectangle(extent={{-80,-60},{80,60}},
                   fillPattern=FillPattern.Solid,
                   fillColor={240,240,240},
                   lineColor={0,0,0}),
         Text(extent={{-70,30},{70,-10}},
              textString="Hydronics+Zone",
              textColor={0,0,0}),
         Text(extent={{-70,-10},{70,-40}},
              textString="(SimpleWater)",
              textColor={0,128,0})}),
  Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
  Documentation(info="<html>
<h4>SystemWithZoneAndHydronics_SimpleWater</h4>
<p>Same as SystemWithZoneAndHydronics but uses ConstantPropertyLiquidWater medium.</p>
<p>This avoids IF97 temperature limit errors that occur when zone temperature
drops below 273.15 K during solver transients.</p>
<p><b>Usage:</b></p>
<pre>
HydronicSystem.SystemWithZoneAndHydronics_SimpleWater cellar_hydSys(
  redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater)
</pre>
</html>"));
end SystemWithZoneAndHydronics_SimpleWater;
