within CoSES_Thermal_ProHMo_PHiL.Data;
record CellarZoneParams
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

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<h4>Cellar Zone Parameters</h4>
<li>
Dec, 2025, by Karthik Murugesan
</li>
<p><b>Added a record for Zone parameters</b></p>
</html>"));
end CellarZoneParams;
