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

end EnvironmentConditions;
