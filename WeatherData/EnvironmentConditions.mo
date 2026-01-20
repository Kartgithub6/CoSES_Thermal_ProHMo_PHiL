within CoSES_Thermal_ProHMo_PHiL.WeatherData;
model EnvironmentConditions
  // Heat exchange interface
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port
    "Ambient heat port";

  // Additional environment parameters
  parameter Real DayOfYear;
  parameter Boolean LeapYear;
  parameter Real cGround;
  parameter Real lambdaGround;
  parameter Real rhoGround;
  parameter Real GeoGradient;
  parameter Real TAmbientAverageAct;
  parameter Real TAverageAmbientAnnual;
  parameter Real TAmbientMax;
  parameter Real TAmbient;
  parameter Integer MaxMonth;
  parameter Real rhoAir = 1.2   "Air density [kg/m3]";
  parameter Real cpAir  = 1005  "Air specific heat [J/kg.K]";
  parameter Real RadiationVector[3];
  parameter Real RadiationDirect "Direct solar radiation [W/m2]"; // removed value 0
  parameter Real RadiationDiffuse "Diffuse solar radiation [W/m2]"; // removed value 0



  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end EnvironmentConditions;
