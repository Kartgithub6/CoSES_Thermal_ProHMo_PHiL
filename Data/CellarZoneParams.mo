within CoSES_Thermal_ProHMo_PHiL.Data;
record CellarZoneParams
  // … your existing parameters …

  // Infiltration / ventilation
  parameter Real infiltrationRate(unit="1/h")
    "Air change rate for infiltration";

  // Heating system
  parameter Modelica.Units.SI.HeatFlowRate   QHeatNorm
    "Nominal heating power of the radiator";
  parameter Modelica.Units.SI.Temperature    TFlowHeatNorm
    "Nominal supply temperature of heating water";
  parameter Modelica.Units.SI.Temperature    TReturnHeatInit
    "Initial/nominal return temperature of heating water";
  parameter Modelica.Units.SI.Volume         VHeatMedium
    "Water volume in the radiator and local pipes";

  // Internal gains
  parameter Modelica.Units.SI.HeatFlowRate   occupancyLoad
    "Heat gains from occupants";
  parameter Modelica.Units.SI.HeatFlowRate   applianceLoad
    "Heat gains from appliances";
end CellarZoneParams;
