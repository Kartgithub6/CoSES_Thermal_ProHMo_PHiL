within CoSES_Thermal_ProHMo_PHiL.Interfaces;
record EnvironmentConditions
  // Air properties
  parameter Modelica.Units.SI.SpecificHeatCapacity cpAir = 1006;
  parameter Modelica.Units.SI.Density rhoAir = 1.2;

  // Ground properties
  parameter Modelica.Units.SI.SpecificHeatCapacity cGround = 800;
  parameter Modelica.Units.SI.Density rhoGround = 1800;
  parameter Modelica.Units.SI.ThermalConductivity lambdaGround = 1.5;
  parameter Real GeoGradient = 0.03;

  // Ambient
  parameter Modelica.Units.SI.Temperature TAmbient = 283.15;
  parameter Modelica.Units.SI.Temperature TAmbientAverageAct = 283.15;
  parameter Modelica.Units.SI.Temperature TAmbientMax = 303.15;
  parameter Modelica.Units.SI.Temperature TAverageAmbientAnnual = 283.15;

  // Solar radiation
  parameter Real RadiationVector[3] = {0,0,1};
  parameter Modelica.Units.SI.HeatFlux RadiationDirect = 0;
  parameter Modelica.Units.SI.HeatFlux RadiationDiffuse = 0;

  // Time
  parameter Integer DayOfYear = 1;

  annotation(
    Icon(
      coordinateSystem(extent={{-100,-100},{100,100}}, preserveAspectRatio=true),
      graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={240,248,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-70,90},{-30,50}},
          lineColor={255,180,0},
          fillColor={255,220,50},
          fillPattern=FillPattern.Solid),
        Line(points={{-50,100},{-50,90}}, color={255,200,0}, thickness=1),
        Line(points={{-50,40},{-50,30}}, color={255,200,0}, thickness=1),
        Line(points={{-80,70},{-90,70}}, color={255,200,0}, thickness=1),
        Line(points={{-20,70},{-10,70}}, color={255,200,0}, thickness=1),
        Line(points={{-65,85},{-72,92}}, color={255,200,0}, thickness=1),
        Line(points={{-35,85},{-28,92}}, color={255,200,0}, thickness=1),
        Line(points={{-65,55},{-72,48}}, color={255,200,0}, thickness=1),
        Line(points={{-35,55},{-28,48}}, color={255,200,0}, thickness=1),
        Ellipse(
          extent={{0,50},{40,20}},
          lineColor={150,150,150},
          fillColor={220,220,220},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{20,60},{50,30}},
          lineColor={150,150,150},
          fillColor={220,220,220},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{30,55},{60,25}},
          lineColor={150,150,150},
          fillColor={220,220,220},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-80,-10},{-80,-60}},
          color={238,46,47},
          thickness=2.5),
        Ellipse(
          extent={{-87,-60},{-73,-74}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,-15},{-75,-15}}, color={0,0,0}, thickness=0.5),
        Line(points={{-80,-30},{-75,-30}}, color={0,0,0}, thickness=0.5),
        Line(points={{-80,-45},{-75,-45}}, color={0,0,0}, thickness=0.5),
        Polygon(
          points={{70,45},{60,40},{70,35},{70,45}},
          lineColor={0,100,200},
          fillColor={0,100,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{30,40},{70,40}},
          color={0,100,200},
          thickness=1.5),
        Rectangle(
          extent={{-100,-85},{100,-100}},
          lineColor={101,67,33},
          fillColor={139,69,19},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,-85},{-85,-90}}, color={34,139,34}, thickness=1),
        Line(points={{-80,-85},{-75,-90}}, color={34,139,34}, thickness=1),
        Line(points={{-40,-85},{-45,-90}}, color={34,139,34}, thickness=1),
        Line(points={{-40,-85},{-35,-90}}, color={34,139,34}, thickness=1),
        Line(points={{0,-85},{-5,-90}}, color={34,139,34}, thickness=1),
        Line(points={{0,-85},{5,-90}}, color={34,139,34}, thickness=1),
        Line(points={{40,-85},{35,-90}}, color={34,139,34}, thickness=1),
        Line(points={{40,-85},{45,-90}}, color={34,139,34}, thickness=1),
        Line(points={{80,-85},{75,-90}}, color={34,139,34}, thickness=1),
        Line(points={{80,-85},{85,-90}}, color={34,139,34}, thickness=1),
        Text(
          extent={{-90,20},{90,0}},
          textString="WEATHER",
          textColor={0,0,0},
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-100,120},{100,160}},
          textString="%name",
          textColor={0,0,255})}),
    Documentation(info="<html>
<h4>Environment Conditions</h4>
<li>
Sep 20, 2025, by Karthik Murugesan
</li>
<p><b>Added new Environment Conditions under Interfaces</b></p>
</html>"));
end EnvironmentConditions;
