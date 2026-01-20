within CoSES_Thermal_ProHMo_PHiL;
package Consumer
  model SimpleDHW
    "Simplified Domestic Hot Water Demand Model for Dymola"

    // ============================================================================
    // MODEL DESCRIPTION
    // ============================================================================
    // Simplified DHW demand model based on SimulationX DHW_demand
    // - Calculates DHW heat demand from volume flow profile
    // - Q_DHW = qv * rho * cp * (T_set - T_cold)
    // - Tracks energy consumption and missing energy
    // ============================================================================

    // ============================================================================
    // PARAMETERS
    // ============================================================================
    parameter Modelica.Units.SI.Temperature THotWaterSet = 318.15
      "Hot water set temperature [K] (45°C default)";

    parameter Modelica.Units.SI.Temperature TColdWaterConst = 283.15
      "Cold water temperature [K] (10°C default)";

    parameter Real DHWfactor = 1.0
      "DHW demand multiplication factor";

    parameter Modelica.Units.SI.TemperatureDifference DeltaT_tolerance = 1
      "Accepted lower tolerance for T_DHW [K]";

    parameter Modelica.Units.SI.SpecificHeatCapacity cpWater = 4180
      "Specific heat capacity of water [J/(kg·K)]";

    parameter Modelica.Units.SI.Density rhoWater = 1000
      "Density of water [kg/m³]";

    // ============================================================================
    // INPUTS
    // ============================================================================
    Modelica.Blocks.Interfaces.RealInput qv_DHW_demand_Lph
      "DHW volume flow demand [L/h]"
      annotation(Placement(transformation(extent={{-140,40},{-100,80}})));

    Modelica.Blocks.Interfaces.RealInput TStorage_degC
      "Storage/supply temperature [°C]"
      annotation(Placement(transformation(extent={{-140,-20},{-100,20}})));

    Modelica.Blocks.Interfaces.RealInput TColdWater_degC(start=10)
      "Cold water temperature [°C] (optional, use constant if not connected)"
      annotation(Placement(transformation(extent={{-140,-80},{-100,-40}})));

    // ============================================================================
    // OUTPUTS
    // ============================================================================
    Modelica.Blocks.Interfaces.RealOutput Q_DHW_W
      "DHW heat power demand [W]"
      annotation(Placement(transformation(extent={{100,50},{120,70}})));

    Modelica.Blocks.Interfaces.RealOutput qv_DHW_actual_Lph
      "Actual DHW volume flow [L/h]"
      annotation(Placement(transformation(extent={{100,10},{120,30}})));

    Modelica.Blocks.Interfaces.RealOutput Q_missing_W
      "Missing heat power [W]"
      annotation(Placement(transformation(extent={{100,-30},{120,-10}})));

    Modelica.Blocks.Interfaces.RealOutput T_fail
      "1 if supply temp too low, 0 otherwise"
      annotation(Placement(transformation(extent={{100,-70},{120,-50}})));

    // ============================================================================
    // INTERNAL VARIABLES
    // ============================================================================
    Modelica.Units.SI.Temperature TStorage "Storage temperature [K]";
    Modelica.Units.SI.Temperature TColdWater "Cold water temperature [K]";
    Modelica.Units.SI.VolumeFlowRate qv_demand "Demand flow [m³/s]";
    Modelica.Units.SI.VolumeFlowRate qv_actual "Actual flow [m³/s]";
    Modelica.Units.SI.Power Q_DHW "DHW heat power [W]";
    Modelica.Units.SI.Power Q_DHW_set "Set heat power [W]";
    Boolean tempFail "Temperature failure flag";

    // ============================================================================
    // CUMULATIVE ENERGY
    // ============================================================================
    Modelica.Units.SI.Energy E_DHW(start=0, fixed=true) "Cumulative DHW energy [J]";
    Modelica.Units.SI.Energy E_missing(start=0, fixed=true) "Cumulative missing energy [J]";
    Real E_DHW_kWh "Cumulative DHW energy [kWh]";
    Real E_missing_kWh "Cumulative missing energy [kWh]";
    Modelica.Units.SI.Volume V_DHW(start=0, fixed=true) "Cumulative DHW volume [m³]";
    Real V_DHW_L "Cumulative DHW volume [L]";

  equation
    // ============================================================================
    // TEMPERATURE CONVERSIONS
    // ============================================================================
    TStorage = TStorage_degC + 273.15;
    TColdWater = TColdWaterConst;  // Use constant cold water temp

    // ============================================================================
    // VOLUME FLOW CONVERSION
    // ============================================================================
    qv_demand = qv_DHW_demand_Lph / 3600000 * DHWfactor;  // L/h to m³/s

    // ============================================================================
    // DHW HEAT DEMAND CALCULATION
    // ============================================================================
    Q_DHW_set = qv_demand * cpWater * rhoWater * (THotWaterSet - TColdWater);

    if qv_demand > 1e-8 then
      if TStorage > (THotWaterSet - DeltaT_tolerance) then
        // Storage can meet demand
        qv_actual = qv_demand;
        Q_DHW = qv_actual * cpWater * rhoWater * (TStorage - TColdWater);
        tempFail = false;
        Q_missing_W = 0;
      else
        // Storage temperature too low
        qv_actual = qv_demand;  // Still flow, but at lower temp
        Q_DHW = qv_actual * cpWater * rhoWater * max(0, TStorage - TColdWater);
        tempFail = true;
        Q_missing_W = Q_DHW_set - Q_DHW;
      end if;
    else
      qv_actual = 0;
      Q_DHW = 0;
      tempFail = false;
      Q_missing_W = 0;
    end if;

    // ============================================================================
    // OUTPUTS
    // ============================================================================
    Q_DHW_W = Q_DHW;
    qv_DHW_actual_Lph = qv_actual * 3600000;  // m³/s to L/h
    T_fail = if tempFail then 1 else 0;

    // ============================================================================
    // CUMULATIVE ENERGY
    // ============================================================================
    der(E_DHW) = Q_DHW;
    der(E_missing) = Q_missing_W;
    der(V_DHW) = qv_actual;

    E_DHW_kWh = E_DHW / 3600000;
    E_missing_kWh = E_missing / 3600000;
    V_DHW_L = V_DHW * 1000;

    annotation(
      Icon(graphics={
        Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0},
          fillColor={255,255,255}, fillPattern=FillPattern.Solid),
        Rectangle(extent={{-20,60},{20,40}}, lineColor={0,0,0},
          fillColor={192,192,192}, fillPattern=FillPattern.Solid),
        Ellipse(extent={{-30,50},{-10,30}}, lineColor={0,0,0},
          fillColor={192,192,192}, fillPattern=FillPattern.Solid),
        Ellipse(extent={{-10,20},{0,0}}, lineColor={0,100,255},
          fillColor={100,150,255}, fillPattern=FillPattern.Solid),
        Ellipse(extent={{0,10},{10,-10}}, lineColor={0,100,255},
          fillColor={100,150,255}, fillPattern=FillPattern.Solid),
        Ellipse(extent={{-5,-5},{5,-20}}, lineColor={0,100,255},
          fillColor={100,150,255}, fillPattern=FillPattern.Solid),
        Rectangle(extent={{-60,-40},{60,-60}}, lineColor={0,0,0},
          fillColor={220,220,220}, fillPattern=FillPattern.Solid),
        Text(extent={{-80,-65},{80,-90}}, textColor={0,0,0},
          textString="DHW"),
        Text(extent={{-100,140},{100,110}}, textColor={0,0,255},
          textString="%name")}

        // Tap/faucet

        // Water drops

        // Sink

),    Documentation(info="<html>
<h4>SimpleDHW - Domestic Hot Water Demand</h4>
<p>Simplified DHW model based on SimulationX DHW_demand.</p>

<h5>Inputs:</h5>
<ul>
<li>qv_DHW_demand_Lph: DHW volume flow demand [L/h]</li>
<li>TStorage_degC: Storage/supply temperature [°C]</li>
<li>TColdWater_degC: Cold water temperature [°C]</li>
</ul>

<h5>Outputs:</h5>
<ul>
<li>Q_DHW_W: DHW heat power [W]</li>
<li>qv_DHW_actual_Lph: Actual volume flow [L/h]</li>
<li>Q_missing_W: Missing heat power [W]</li>
<li>T_fail: Temperature failure flag</li>
</ul>

<h5>Parameters:</h5>
<ul>
<li>THotWaterSet: Hot water set temperature (default 45°C)</li>
<li>TColdWaterConst: Cold water temperature (default 10°C)</li>
<li>DHWfactor: Demand multiplication factor</li>
</ul>
</html>"));
  end SimpleDHW;
end Consumer;
