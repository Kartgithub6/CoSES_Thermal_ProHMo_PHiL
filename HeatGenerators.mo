within CoSES_Thermal_ProHMo_PHiL;
package HeatGenerators
  model SimpleCHP
    "Simplified CHP (Combined Heat and Power) model for Dymola - based on SimulationX physics"

    // ============================================================================
    // DESCRIPTION
    // ============================================================================
    // This is a simplified CHP model that:
    // - Produces both heat (QHeat) and electricity (PElectrical)
    // - Has modulating power output (30-100%)
    // - Calculates supply temperature based on heat added to water flow
    // - Includes thermal dynamics (first-order response)
    //
    // Based on SimulationX GreenCity CHP model physics
    // ============================================================================

    // ============================================================================
    // PARAMETERS - CHP Specifications
    // ============================================================================

    parameter Modelica.Units.SI.Power QHeat_nominal = 12500
      "Nominal thermal power output [W] (e.g., 12.5 kW)"
      annotation(Dialog(group="Power Rating"));

    parameter Modelica.Units.SI.Power PElec_nominal = 5000
      "Nominal electrical power output [W] (e.g., 5 kW)"
      annotation(Dialog(group="Power Rating"));

    parameter Real eta_heat_nominal = 0.60
      "Nominal thermal efficiency [-] (typically 0.55-0.65)"
      annotation(Dialog(group="Efficiency"));

    parameter Real eta_elec_nominal = 0.25
      "Nominal electrical efficiency [-] (typically 0.20-0.30)"
      annotation(Dialog(group="Efficiency"));

    parameter Real ModulationMin = 0.30
      "Minimum modulation (30% = cannot go below 30% power)"
      annotation(Dialog(group="Control"));

    parameter Modelica.Units.SI.Temperature TFlow_max = 363.15
      "Maximum flow temperature [K] (90°C)"
      annotation(Dialog(group="Temperature Limits"));

    parameter Modelica.Units.SI.Time tau = 60
      "Thermal time constant [s] (how fast CHP responds)"
      annotation(Dialog(group="Dynamics"));

    // Medium properties (water)
    parameter Modelica.Units.SI.SpecificHeatCapacity cp = 4180
      "Specific heat capacity of water [J/(kg·K)]";
    parameter Modelica.Units.SI.Density rho = 1000
      "Density of water [kg/m³]";

    // ============================================================================
    // INPUTS
    // ============================================================================

    Modelica.Blocks.Interfaces.BooleanInput CHPon
      "CHP on/off command"
      annotation(Placement(transformation(extent={{-140,60},{-100,100}}),
          iconTransformation(extent={{-140,60},{-100,100}})));

    Modelica.Blocks.Interfaces.RealInput Modulation(min=0, max=1)
      "Power modulation signal [0-1] (0=min power, 1=max power)"
      annotation(Placement(transformation(extent={{-140,10},{-100,50}}),
          iconTransformation(extent={{-140,10},{-100,50}})));

    Modelica.Blocks.Interfaces.RealInput TReturn_degC
      "Return water temperature [°C]"
      annotation(Placement(transformation(extent={{-140,-50},{-100,-10}}),
          iconTransformation(extent={{-140,-50},{-100,-10}})));

    Modelica.Blocks.Interfaces.RealInput qv_l_per_min
      "Volume flow rate [L/min]"
      annotation(Placement(transformation(extent={{-140,-100},{-100,-60}}),
          iconTransformation(extent={{-140,-100},{-100,-60}})));

    // ============================================================================
    // OUTPUTS
    // ============================================================================

    Modelica.Blocks.Interfaces.RealOutput TSupply_degC
      "Supply water temperature [°C]"
      annotation(Placement(transformation(extent={{100,60},{120,80}}),
          iconTransformation(extent={{100,60},{120,80}})));

    Modelica.Blocks.Interfaces.RealOutput QHeat_kW
      "Actual thermal power output [kW]"
      annotation(Placement(transformation(extent={{100,20},{120,40}}),
          iconTransformation(extent={{100,20},{120,40}})));

    Modelica.Blocks.Interfaces.RealOutput PElec_kW
      "Actual electrical power output [kW]"
      annotation(Placement(transformation(extent={{100,-20},{120,0}}),
          iconTransformation(extent={{100,-20},{120,0}})));

    Modelica.Blocks.Interfaces.RealOutput PFuel_kW
      "Fuel power consumption [kW]"
      annotation(Placement(transformation(extent={{100,-60},{120,-40}}),
          iconTransformation(extent={{100,-60},{120,-40}})));

    Modelica.Blocks.Interfaces.RealOutput Efficiency
      "Overall efficiency (heat+elec)/fuel [-]"
      annotation(Placement(transformation(extent={{100,-100},{120,-80}}),
          iconTransformation(extent={{100,-100},{120,-80}})));

    // ============================================================================
    // INTERNAL VARIABLES
    // ============================================================================

  protected
    Real ModulationActual "Actual modulation (limited to min-max range)";
    Modelica.Units.SI.Power QHeat_target "Target heat power [W]";
    Modelica.Units.SI.Power QHeat_actual(start=0) "Actual heat power [W]";
    Modelica.Units.SI.Power PElec_actual "Actual electrical power [W]";
    Modelica.Units.SI.Power PFuel_actual "Actual fuel power [W]";
    Modelica.Units.SI.Temperature TReturn_K "Return temperature [K]";
    Modelica.Units.SI.Temperature TSupply_K "Supply temperature [K]";
    Modelica.Units.SI.VolumeFlowRate qv "Volume flow [m³/s]";
    Modelica.Units.SI.MassFlowRate m_flow "Mass flow [kg/s]";
    Real eta_heat "Actual thermal efficiency";
    Real eta_elec "Actual electrical efficiency";

  equation
    // ============================================================================
    // UNIT CONVERSIONS
    // ============================================================================
    TReturn_K = TReturn_degC + 273.15;
    qv = qv_l_per_min / 60000;  // L/min to m³/s
    m_flow = rho * qv;

    // ============================================================================
    // MODULATION LOGIC
    // ============================================================================
    // When ON: modulation is limited between ModulationMin and 1.0
    // When OFF: modulation is 0
    ModulationActual = if CHPon then max(ModulationMin, min(1.0, Modulation)) else 0;

    // ============================================================================
    // EFFICIENCY CALCULATION
    // ============================================================================
    // Efficiency improves slightly at lower return temperatures (condensing effect)
    // Simplified linear model: eta = eta_nominal * (1 + 0.05 * (50 - TReturn_degC) / 50)
    // This gives ~5% boost at 0°C return, nominal at 50°C return
    eta_heat = eta_heat_nominal * (1 + 0.05 * max(0, min(1, (323.15 - TReturn_K) / 50)));
    eta_elec = eta_elec_nominal;  // Electrical efficiency relatively constant

    // ============================================================================
    // POWER CALCULATIONS
    // ============================================================================
    QHeat_target = ModulationActual * QHeat_nominal;

    // First-order dynamics for thermal response
    tau * der(QHeat_actual) = QHeat_target - QHeat_actual;

    // Electrical power proportional to thermal power
    PElec_actual = (QHeat_actual / QHeat_nominal) * PElec_nominal;

    // Fuel power from efficiency
    PFuel_actual = if CHPon and (eta_heat + eta_elec) > 0.01 then
                     (QHeat_actual + PElec_actual) / (eta_heat + eta_elec)
                   else 0;

    // ============================================================================
    // SUPPLY TEMPERATURE CALCULATION
    // ============================================================================
    // Q = m_flow * cp * (TSupply - TReturn)
    // TSupply = TReturn + Q / (m_flow * cp)

    if m_flow > 1e-6 then
      TSupply_K = min(TFlow_max, TReturn_K + QHeat_actual / (m_flow * cp));
    else
      // No flow: temperature rises to max (safety limit)
      TSupply_K = if CHPon then TFlow_max else TReturn_K;
    end if;

    // ============================================================================
    // OUTPUTS
    // ============================================================================
    TSupply_degC = TSupply_K - 273.15;
    QHeat_kW = QHeat_actual / 1000;
    PElec_kW = PElec_actual / 1000;
    PFuel_kW = PFuel_actual / 1000;
    Efficiency = if PFuel_actual > 100 then (QHeat_actual + PElec_actual) / PFuel_actual else 0;

    annotation(
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
        graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-80,80},{80,-60}},
            lineColor={255,128,0},
            fillColor={255,200,150},
            fillPattern=FillPattern.Solid,
            lineThickness=1),
          Text(
            extent={{-60,60},{60,20}},
            textColor={255,128,0},
            textString="CHP",
            textStyle={TextStyle.Bold}),
          Rectangle(
            extent={{-60,10},{-20,-30}},
            lineColor={255,0,0},
            fillColor={255,100,100},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-55,5},{-25,-25}},
            textColor={255,255,255},
            textString="Q"),
          Rectangle(
            extent={{20,10},{60,-30}},
            lineColor={255,215,0},
            fillColor={255,255,100},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{25,5},{55,-25}},
            textColor={100,100,0},
            textString="P"),
          Line(points={{0,-60},{0,-80}}, color={0,0,255}, thickness=1),
          Line(points={{-40,-60},{-40,-80}}, color={0,0,255}, thickness=1),
          Line(points={{40,-60},{40,-80}}, color={255,0,0}, thickness=1),
          Text(
            extent={{-60,-45},{60,-58}},
            textColor={0,0,0},
            textString="Heat + Power"),
          Text(
            extent={{-100,100},{100,82}},
            textColor={0,0,255},
            textString="%name")}),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}})),
      Documentation(info="<html>
<h4>SimpleCHP - Combined Heat and Power Unit</h4>
<p>Simplified CHP model based on SimulationX GreenCity physics.</p>

<h5>Features:</h5>
<ul>
<li>Produces both thermal and electrical power</li>
<li>Modulating output (30-100% of nominal)</li>
<li>First-order thermal dynamics</li>
<li>Efficiency depends on return temperature</li>
</ul>

<h5>Typical Values:</h5>
<ul>
<li>QHeat_nominal: 12.5 kW thermal</li>
<li>PElec_nominal: 5 kW electrical</li>
<li>eta_heat: 60%</li>
<li>eta_elec: 25%</li>
<li>Total efficiency: 85%</li>
</ul>

<h5>Usage:</h5>
<p>Connect TReturn_degC from building return, get TSupply_degC to building supply.</p>
</html>"));
  end SimpleCHP;

  model SimpleCondensingBoiler
    "Simplified Condensing Boiler model for Dymola - based on SimulationX physics"

    // ============================================================================
    // DESCRIPTION
    // ============================================================================
    // This is a simplified condensing boiler model that:
    // - Produces heat only (no electricity)
    // - Has modulating power output (20-100%)
    // - Higher efficiency at lower return temperatures (condensing effect)
    // - Includes thermal mass dynamics
    //
    // Based on SimulationX GreenCity CondensingBoiler model physics
    // ============================================================================

    // ============================================================================
    // PARAMETERS - Boiler Specifications
    // ============================================================================

    parameter Modelica.Units.SI.Power QHeat_nominal = 50000
      "Nominal thermal power output [W] (e.g., 50 kW)"
      annotation(Dialog(group="Power Rating"));

    parameter Real eta_nominal = 0.98
      "Nominal efficiency at low return temp [-] (condensing: 0.95-1.09)"
      annotation(Dialog(group="Efficiency"));

    parameter Real eta_noncondensing = 0.88
      "Efficiency at high return temp (non-condensing) [-]"
      annotation(Dialog(group="Efficiency"));

    parameter Modelica.Units.SI.Temperature T_condensing = 328.15
      "Return temp below which condensing occurs [K] (55°C)"
      annotation(Dialog(group="Efficiency"));

    parameter Real ModulationMin = 0.20
      "Minimum modulation (20% = cannot go below 20% power)"
      annotation(Dialog(group="Control"));

    parameter Modelica.Units.SI.Temperature TFlow_max = 358.15
      "Maximum flow temperature [K] (85°C)"
      annotation(Dialog(group="Temperature Limits"));

    parameter Modelica.Units.SI.Volume V_boiler = 0.010
      "Water volume in boiler [m³] (10 liters typical)"
      annotation(Dialog(group="Thermal Mass"));

    parameter Real QlossRate(unit="W/K") = 50
      "Standby heat loss coefficient [W/K]"
      annotation(Dialog(group="Losses"));

    parameter Modelica.Units.SI.Temperature TAmbient = 293.15
      "Ambient temperature around boiler [K] (20°C)"
      annotation(Dialog(group="Losses"));

    parameter Modelica.Units.SI.Time tau = 30
      "Thermal time constant [s]"
      annotation(Dialog(group="Dynamics"));

    // Medium properties (water)
    parameter Modelica.Units.SI.SpecificHeatCapacity cp = 4180
      "Specific heat capacity of water [J/(kg·K)]";
    parameter Modelica.Units.SI.Density rho = 1000
      "Density of water [kg/m³]";

    // ============================================================================
    // INPUTS
    // ============================================================================

    Modelica.Blocks.Interfaces.BooleanInput CBon
      "Boiler on/off command"
      annotation(Placement(transformation(extent={{-140,60},{-100,100}}),
          iconTransformation(extent={{-140,60},{-100,100}})));

    Modelica.Blocks.Interfaces.RealInput Modulation(min=0, max=1)
      "Power modulation signal [0-1] (0=min power, 1=max power)"
      annotation(Placement(transformation(extent={{-140,10},{-100,50}}),
          iconTransformation(extent={{-140,10},{-100,50}})));

    Modelica.Blocks.Interfaces.RealInput TReturn_degC
      "Return water temperature [°C]"
      annotation(Placement(transformation(extent={{-140,-50},{-100,-10}}),
          iconTransformation(extent={{-140,-50},{-100,-10}})));

    Modelica.Blocks.Interfaces.RealInput qv_l_per_min
      "Volume flow rate [L/min]"
      annotation(Placement(transformation(extent={{-140,-100},{-100,-60}}),
          iconTransformation(extent={{-140,-100},{-100,-60}})));

    // ============================================================================
    // OUTPUTS
    // ============================================================================

    Modelica.Blocks.Interfaces.RealOutput TSupply_degC
      "Supply water temperature [°C]"
      annotation(Placement(transformation(extent={{100,60},{120,80}}),
          iconTransformation(extent={{100,60},{120,80}})));

    Modelica.Blocks.Interfaces.RealOutput QHeat_kW
      "Actual thermal power output [kW]"
      annotation(Placement(transformation(extent={{100,20},{120,40}}),
          iconTransformation(extent={{100,20},{120,40}})));

    Modelica.Blocks.Interfaces.RealOutput PFuel_kW
      "Fuel (gas) power consumption [kW]"
      annotation(Placement(transformation(extent={{100,-20},{120,0}}),
          iconTransformation(extent={{100,-20},{120,0}})));

    Modelica.Blocks.Interfaces.RealOutput Efficiency
      "Actual efficiency [-]"
      annotation(Placement(transformation(extent={{100,-60},{120,-40}}),
          iconTransformation(extent={{100,-60},{120,-40}})));

    // ============================================================================
    // INTERNAL VARIABLES
    // ============================================================================

  protected
    Real ModulationActual "Actual modulation (limited to min-max range)";
    Modelica.Units.SI.Power QHeat_target "Target heat power [W]";
    Modelica.Units.SI.Power QHeat_actual(start=0) "Actual heat power [W]";
    Modelica.Units.SI.Power PFuel_actual "Actual fuel power [W]";
    Modelica.Units.SI.Temperature TReturn_K "Return temperature [K]";
    Modelica.Units.SI.Temperature TSupply_K(start=333.15) "Supply temperature [K]";
    Modelica.Units.SI.VolumeFlowRate qv "Volume flow [m³/s]";
    Modelica.Units.SI.MassFlowRate m_flow "Mass flow [kg/s]";
    Real eta "Actual efficiency";
    Real condensing_factor "Factor for condensing operation [0-1]";

  equation
    // ============================================================================
    // UNIT CONVERSIONS
    // ============================================================================
    TReturn_K = TReturn_degC + 273.15;
    qv = qv_l_per_min / 60000;  // L/min to m³/s
    m_flow = rho * qv;

    // ============================================================================
    // MODULATION LOGIC
    // ============================================================================
    ModulationActual = if CBon then max(ModulationMin, min(1.0, Modulation)) else 0;

    // ============================================================================
    // EFFICIENCY CALCULATION - CONDENSING EFFECT
    // ============================================================================
    // Efficiency varies between eta_noncondensing (high return temp)
    // and eta_nominal (low return temp, condensing mode)
    // Transition occurs around T_condensing (typically 55°C)

    condensing_factor = max(0, min(1, (T_condensing - TReturn_K) / 20));
    // condensing_factor = 1 when TReturn < 35°C, = 0 when TReturn > 55°C

    eta = eta_noncondensing + condensing_factor * (eta_nominal - eta_noncondensing);

    // ============================================================================
    // POWER AND TEMPERATURE CALCULATIONS
    // ============================================================================
    QHeat_target = ModulationActual * QHeat_nominal;

    // First-order dynamics for heat output
    tau * der(QHeat_actual) = QHeat_target - QHeat_actual;

    // Fuel consumption
    PFuel_actual = if CBon and eta > 0.01 then QHeat_actual / eta else 0;

    // ============================================================================
    // SUPPLY TEMPERATURE - With thermal mass dynamics
    // ============================================================================
    // Energy balance on boiler water volume:
    // rho * V * cp * dT/dt = QHeat - Qloss - Qflow
    // where Qflow = m_flow * cp * (TSupply - TReturn)

    rho * V_boiler * cp * der(TSupply_K) =
      QHeat_actual
      - QlossRate * (TSupply_K - TAmbient)
      - m_flow * cp * (TSupply_K - TReturn_K);        // Heat input from burner
                                                      // Heat loss to ambient
                                                      // Heat delivered to flow

    // Limit supply temperature to maximum
    // Note: This is handled by the differential equation naturally if QHeat reduces

    // ============================================================================
    // OUTPUTS
    // ============================================================================
    TSupply_degC = min(TFlow_max, TSupply_K) - 273.15;
    QHeat_kW = QHeat_actual / 1000;
    PFuel_kW = PFuel_actual / 1000;
    Efficiency = eta;

  initial equation
    TSupply_K = TReturn_K + 10;  // Start 10K above return
    QHeat_actual = 0;

    annotation(
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
        graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-80,80},{80,-60}},
            lineColor={0,0,255},
            fillColor={200,200,255},
            fillPattern=FillPattern.Solid,
            lineThickness=1),
          Text(
            extent={{-60,60},{60,20}},
            textColor={0,0,200},
            textString="CB",
            textStyle={TextStyle.Bold}),
          Polygon(
            points={{-40,-10},{0,30},{40,-10},{20,-10},{20,-40},{-20,-40},{-20,-10},{-40,-10}},
            lineColor={255,100,0},
            fillColor={255,150,50},
            fillPattern=FillPattern.Solid),
          Line(points={{-50,-60},{-50,-80}}, color={0,0,255}, thickness=1),
          Line(points={{50,-60},{50,-80}}, color={255,0,0}, thickness=1),
          Text(
            extent={{-60,-45},{60,-58}},
            textColor={0,0,0},
            textString="Condensing"),
          Text(
            extent={{-100,100},{100,82}},
            textColor={0,0,255},
            textString="%name"),
          Ellipse(
            extent={{-30,-20},{-10,-40}},
            lineColor={255,200,0},
            fillColor={255,255,0},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{10,-20},{30,-40}},
            lineColor={255,200,0},
            fillColor={255,255,0},
            fillPattern=FillPattern.Solid)}),
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}})),
      Documentation(info="<html>
<h4>SimpleCondensingBoiler - Gas Condensing Boiler</h4>
<p>Simplified condensing boiler model based on SimulationX GreenCity physics.</p>

<h5>Features:</h5>
<ul>
<li>Modulating heat output (20-100% of nominal)</li>
<li>Condensing efficiency at low return temperatures</li>
<li>Thermal mass dynamics (boiler water volume)</li>
<li>Standby heat losses</li>
</ul>

<h5>Typical Values:</h5>
<ul>
<li>QHeat_nominal: 50 kW (for large house/small apartment building)</li>
<li>eta_nominal: 98% (condensing mode, return &lt; 35°C)</li>
<li>eta_noncondensing: 88% (non-condensing, return &gt; 55°C)</li>
</ul>

<h5>Condensing Effect:</h5>
<p>When return water temperature is below ~55°C, water vapor in flue gas 
condenses, recovering latent heat and boosting efficiency above 100% 
(relative to lower heating value).</p>

<h5>Usage:</h5>
<p>Connect TReturn_degC from building return, get TSupply_degC to building supply.</p>
</html>"));
  end SimpleCondensingBoiler;

  model SimpleCHP_v2
    "Simplified CHP model v2 - with debug outputs and verified physics"

    // ============================================================================
    // PARAMETERS
    // ============================================================================

    parameter Modelica.Units.SI.Power QHeat_nominal = 12500
      "Nominal thermal power output [W]"
      annotation(Dialog(group="Power Rating"));

    parameter Modelica.Units.SI.Power PElec_nominal = 5000
      "Nominal electrical power output [W]"
      annotation(Dialog(group="Power Rating"));

    parameter Real eta_heat = 0.60 "Thermal efficiency [-]"
      annotation(Dialog(group="Efficiency"));

    parameter Real eta_elec = 0.25 "Electrical efficiency [-]"
      annotation(Dialog(group="Efficiency"));

    parameter Real ModulationMin = 0.30 "Minimum modulation (30%)"
      annotation(Dialog(group="Control"));

    parameter Modelica.Units.SI.Temperature TFlow_max = 363.15
      "Maximum flow temperature [K] (90°C)"
      annotation(Dialog(group="Limits"));

    parameter Modelica.Units.SI.Time tau = 60 "Time constant [s]"
      annotation(Dialog(group="Dynamics"));

    // Fixed medium properties
    parameter Real cp = 4180 "Specific heat [J/(kg·K)]";
    parameter Real rho = 1000 "Density [kg/m³]";

    // ============================================================================
    // INPUTS
    // ============================================================================

    Modelica.Blocks.Interfaces.BooleanInput CHPon "CHP on/off"
      annotation(Placement(transformation(extent={{-140,60},{-100,100}})));

    Modelica.Blocks.Interfaces.RealInput Modulation "Modulation [0-1]"
      annotation(Placement(transformation(extent={{-140,10},{-100,50}})));

    Modelica.Blocks.Interfaces.RealInput TReturn_degC "Return temp [°C]"
      annotation(Placement(transformation(extent={{-140,-50},{-100,-10}})));

    Modelica.Blocks.Interfaces.RealInput qv_l_per_min "Flow [L/min]"
      annotation(Placement(transformation(extent={{-140,-100},{-100,-60}})));

    // ============================================================================
    // OUTPUTS
    // ============================================================================

    Modelica.Blocks.Interfaces.RealOutput TSupply_degC "Supply temp [°C]"
      annotation(Placement(transformation(extent={{100,60},{120,80}})));

    Modelica.Blocks.Interfaces.RealOutput QHeat_kW "Heat output [kW]"
      annotation(Placement(transformation(extent={{100,20},{120,40}})));

    Modelica.Blocks.Interfaces.RealOutput PElec_kW "Electrical output [kW]"
      annotation(Placement(transformation(extent={{100,-20},{120,0}})));

    // ============================================================================
    // DEBUG OUTPUTS - Check these in simulation!
    // ============================================================================

    Real debug_m_flow_kg_s "Mass flow [kg/s] - should be ~0.1 for 6 L/min";
    Real debug_deltaT_K "Temperature rise [K] - should be ~30K at full power";
    Real debug_ModActual "Actual modulation - should be 0.3-1.0 when ON";
    Real debug_QHeat_W "Heat power [W] - should be up to 12500W";

    // ============================================================================
    // STATE VARIABLES
    // ============================================================================

  protected
    Modelica.Units.SI.Power QHeat_actual(start=0, fixed=true) "Actual heat [W]";

  equation
    // ============================================================================
    // FLOW CONVERSION
    // ============================================================================
    debug_m_flow_kg_s = rho * (qv_l_per_min / 60000);  // L/min → m³/s → kg/s

    // ============================================================================
    // MODULATION
    // ============================================================================
    debug_ModActual = if CHPon then max(ModulationMin, min(1.0, Modulation)) else 0;

    // ============================================================================
    // HEAT DYNAMICS
    // ============================================================================
    tau * der(QHeat_actual) = debug_ModActual * QHeat_nominal - QHeat_actual;

    debug_QHeat_W = QHeat_actual;

    // ============================================================================
    // TEMPERATURE CALCULATION
    // ============================================================================
    // deltaT = Q / (m_flow * cp)

    if debug_m_flow_kg_s > 0.001 then
      debug_deltaT_K = QHeat_actual / (debug_m_flow_kg_s * cp);
    else
      debug_deltaT_K = 0;
    end if;

    TSupply_degC = min(TFlow_max - 273.15, TReturn_degC + debug_deltaT_K);

    // ============================================================================
    // OUTPUTS
    // ============================================================================
    QHeat_kW = QHeat_actual / 1000;
    PElec_kW = (QHeat_actual / QHeat_nominal) * PElec_nominal / 1000;

    annotation(
      Icon(graphics={
        Rectangle(extent={{-100,100},{100,-100}}, fillColor={255,200,150},
          fillPattern=FillPattern.Solid, lineColor={255,128,0}),
        Text(extent={{-80,40},{80,-40}}, textString="CHP v2",
          textColor={255,128,0}, textStyle={TextStyle.Bold}),
        Text(extent={{-100,100},{100,80}}, textString="%name", textColor={0,0,255})}),
      Documentation(info="<html>
<h4>SimpleCHP_v2 - Debuggable Version</h4>
<p>Check these debug outputs:</p>
<ul>
<li><b>debug_m_flow_kg_s</b>: Should be ~0.1 for 6 L/min</li>
<li><b>debug_ModActual</b>: Should be 0.3-1.0 when ON</li>
<li><b>debug_QHeat_W</b>: Should be up to 12500 W</li>
<li><b>debug_deltaT_K</b>: Should be ~30K at full power with 6 L/min</li>
</ul>
</html>"));
  end SimpleCHP_v2;

  model SimpleSolarThermal
    "Simplified Solar Thermal Collector Model for Dymola"

    // ============================================================================
    // MODEL DESCRIPTION
    // ============================================================================
    // Simplified solar thermal collector based on EN 12975 efficiency model:
    //   η = η₀ - a₁*(Tm-Ta)/G - a₂*(Tm-Ta)²/G
    //
    // Where:
    //   η₀ = Optical efficiency (zero-loss efficiency)
    //   a₁ = First-order heat loss coefficient [W/(m²·K)]
    //   a₂ = Second-order heat loss coefficient [W/(m²·K²)]
    //   Tm = Mean collector temperature [K]
    //   Ta = Ambient temperature [K]
    //   G  = Solar irradiance [W/m²]
    //
    // Based on Wolf CRK12 CPC collector from SimulationX SF1 model
    // ============================================================================

    // ============================================================================
    // PARAMETERS - Collector Properties
    // ============================================================================
    parameter Boolean CPC = true
      "If true: CPC collector, if false: Flat plate collector";

    parameter Modelica.Units.SI.Area AModule = 1.312
      "Effective aperture area per module [m²]";

    parameter Integer nSeries = 1
      "Number of collectors in series";

    parameter Integer nParallel = 1
      "Number of collectors in parallel";

    parameter Modelica.Units.SI.Area ACollector = AModule * nSeries * nParallel
      "Total collector area [m²]";

    parameter Modelica.Units.SI.Volume VAbsorber = 0.0017
      "Absorber volume per module [m³]";

    // ============================================================================
    // PARAMETERS - Optical & Thermal Properties (Wolf CRK12 CPC defaults)
    // ============================================================================
    parameter Real etaOptical = 0.642
      "Optical efficiency η₀ (zero-loss)";

    parameter Real a1 = 0.885
      "First-order loss coefficient [W/(m²·K)]";

    parameter Real a2 = 0.001
      "Second-order loss coefficient [W/(m²·K²)]";

    parameter Modelica.Units.SI.HeatCapacity CCollector = 8416
      "Thermal capacity of collector [J/K]";

    // ============================================================================
    // PARAMETERS - Orientation
    // ============================================================================
    parameter Modelica.Units.SI.Angle alphaModule = 0.611
      "Inclination angle (35° default) [rad]";

    parameter Modelica.Units.SI.Angle betaModule = Modelica.Constants.pi
      "Orientation angle (South = π) [rad]";

    // ============================================================================
    // PARAMETERS - Medium Properties
    // ============================================================================
    parameter Modelica.Units.SI.SpecificHeatCapacity cpMed = 3800
      "Specific heat capacity of solar fluid [J/(kg·K)]";

    parameter Modelica.Units.SI.Density rhoMed = 1040
      "Density of solar fluid [kg/m³]";

    // ============================================================================
    // PARAMETERS - Circulation Pump
    // ============================================================================
    parameter Modelica.Units.SI.VolumeFlowRate qvMax = 0.00033
      "Maximum volume flow rate [m³/s] (20 L/min)";

    parameter Modelica.Units.SI.VolumeFlowRate qvMin = 0.00005
      "Minimum volume flow rate [m³/s] (3 L/min)";

    parameter Modelica.Units.SI.Time tauPump = 10
      "Pump time constant [s]";

    // ============================================================================
    // PARAMETERS - Initial Conditions
    // ============================================================================
    parameter Modelica.Units.SI.Temperature TCollectorInit = 293.15
      "Initial collector temperature [K]";

    // ============================================================================
    // CONNECTORS - Inputs
    // ============================================================================
    Modelica.Blocks.Interfaces.BooleanInput STon
      "Solar thermal pump on/off signal"
      annotation(Placement(transformation(extent={{-140,60},{-100,100}})));

    Modelica.Blocks.Interfaces.RealInput qvRef(unit="m3/s")
      "Reference volume flow rate [m³/s]"
      annotation(Placement(transformation(extent={{-140,10},{-100,50}})));

    Modelica.Blocks.Interfaces.RealInput TReturn_degC
      "Return temperature from storage [°C]"
      annotation(Placement(transformation(extent={{-140,-50},{-100,-10}})));

    Modelica.Blocks.Interfaces.RealInput TAmbient_degC
      "Ambient temperature [°C]"
      annotation(Placement(transformation(extent={{-140,-100},{-100,-60}})));

    Modelica.Blocks.Interfaces.RealInput GHI(unit="W/m2")
      "Global Horizontal Irradiance [W/m²]"
      annotation(Placement(transformation(extent={{-140,-150},{-100,-110}})));

    // ============================================================================
    // CONNECTORS - Outputs
    // ============================================================================
    Modelica.Blocks.Interfaces.RealOutput TFlow_degC
      "Flow temperature to storage [°C]"
      annotation(Placement(transformation(extent={{100,70},{120,90}})));

    Modelica.Blocks.Interfaces.RealOutput TCollector_degC
      "Collector temperature [°C]"
      annotation(Placement(transformation(extent={{100,30},{120,50}})));

    Modelica.Blocks.Interfaces.RealOutput QSolarThermal(unit="W")
      "Thermal power output [W]"
      annotation(Placement(transformation(extent={{100,-10},{120,10}})));

    Modelica.Blocks.Interfaces.RealOutput qvActual(unit="m3/s")
      "Actual volume flow rate [m³/s]"
      annotation(Placement(transformation(extent={{100,-50},{120,-30}})));

    // ============================================================================
    // INTERNAL VARIABLES
    // ============================================================================
    Modelica.Units.SI.Temperature TCollector(start=TCollectorInit, fixed=true)
      "Collector temperature [K]";

    Modelica.Units.SI.Temperature TFlow(start=TCollectorInit, fixed=true)
      "Flow temperature [K]";

    Modelica.Units.SI.Temperature TReturn
      "Return temperature [K]";

    Modelica.Units.SI.Temperature TAmbient
      "Ambient temperature [K]";

    Modelica.Units.SI.Temperature TMean
      "Mean collector temperature [K]";

    Modelica.Units.SI.VolumeFlowRate qv(start=0, fixed=true)
      "Actual volume flow [m³/s]";

    Real eta "Instantaneous collector efficiency";

    Modelica.Units.SI.Irradiance G_eff
      "Effective irradiance on tilted surface [W/m²]";

    Modelica.Units.SI.Power QSolar
      "Solar thermal power [W]";

    Real deltaT "Temperature difference Tm - Ta [K]";

    // ============================================================================
    // CUMULATIVE ENERGY
    // ============================================================================
    Modelica.Units.SI.Energy EST(start=0, fixed=true)
      "Cumulative solar thermal energy [J]";

    Real EST_kWh "Cumulative energy [kWh]";

    // ============================================================================
    // DEBUG VARIABLES
    // ============================================================================
    Real debug_eta "Debug: efficiency";
    Real debug_G_eff "Debug: effective irradiance";
    Real debug_deltaT "Debug: temperature difference";

  equation
    // ============================================================================
    // TEMPERATURE CONVERSIONS
    // ============================================================================
    TReturn = TReturn_degC + 273.15;
    TAmbient = TAmbient_degC + 273.15;
    TMean = (TCollector + TReturn) / 2;
    deltaT = TMean - TAmbient;

    TFlow_degC = TFlow - 273.15;
    TCollector_degC = TCollector - 273.15;

    // ============================================================================
    // EFFECTIVE IRRADIANCE ON TILTED SURFACE
    // ============================================================================
    // Simplified model: use GHI with inclination factor
    // For more accuracy, would need direct/diffuse split and IAM
    G_eff = GHI * (1 + cos(alphaModule)) / 2 * 1.1;  // Approximate boost for tilt

    // ============================================================================
    // COLLECTOR EFFICIENCY (EN 12975)
    // ============================================================================
    if G_eff > 50 then
      // Standard efficiency equation
      eta = max(0, etaOptical - a1 * deltaT / max(G_eff, 1) - a2 * deltaT^2 / max(G_eff, 1));
    else
      eta = 0;
    end if;

    // ============================================================================
    // PUMP DYNAMICS
    // ============================================================================
    if STon then
      tauPump * der(qv) + qv = max(qvMin, min(qvMax, qvRef));
    else
      tauPump * der(qv) + qv = 0;
    end if;

    qvActual = qv;

    // ============================================================================
    // COLLECTOR THERMAL DYNAMICS
    // ============================================================================
    // Energy balance on collector:
    // C * dT/dt = Q_solar_absorbed - Q_losses - Q_removed_by_fluid

    CCollector * der(TCollector) =
      G_eff * ACollector * etaOptical
      - a1 * ACollector * (TCollector - TAmbient)
      - a2 * ACollector * (TCollector - TAmbient)^2
      - cpMed * rhoMed * qv * (TFlow - TReturn);                // Solar input
                                                                // Linear losses
                                                                // Quadratic losses
                                                                // Heat removed

    // ============================================================================
    // FLOW TEMPERATURE DYNAMICS
    // ============================================================================
    // Simplified: Flow temperature tracks collector with small lag
    cpMed * rhoMed * VAbsorber * nSeries * der(TFlow) =
      1000 * (TCollector - TFlow)
      - cpMed * rhoMed * qv * (TFlow - TReturn);                // Heat transfer from absorber
                                                                // Heat removed by flow

    // ============================================================================
    // THERMAL POWER OUTPUT
    // ============================================================================
    QSolar = cpMed * rhoMed * qv * nParallel * (TFlow - TReturn);
    QSolarThermal = max(0, QSolar);

    // ============================================================================
    // CUMULATIVE ENERGY
    // ============================================================================
    der(EST) = QSolarThermal;
    EST_kWh = EST / 3600000;

    // ============================================================================
    // DEBUG OUTPUTS
    // ============================================================================
    debug_eta = eta;
    debug_G_eff = G_eff;
    debug_deltaT = deltaT;

    annotation(
      Icon(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
        graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0},
            fillColor={255,255,255}, fillPattern=FillPattern.Solid),
          Polygon(points={{-80,60},{80,60},{60,-40},{-60,-40},{-80,60}},
            lineColor={0,0,0}, fillColor={0,0,128}, fillPattern=FillPattern.Solid),
          Line(points={{-60,40},{40,40}}, color={255,255,255}),
          Line(points={{-50,20},{50,20}}, color={255,255,255}),
          Line(points={{-40,0},{60,0}}, color={255,255,255}),
          Line(points={{-30,-20},{70,-20}}, color={255,255,255}),
          Line(points={{-40,60},{-60,-40}}, color={255,255,255}),
          Line(points={{0,60},{-20,-40}}, color={255,255,255}),
          Line(points={{40,60},{20,-40}}, color={255,255,255}),
          Ellipse(extent={{50,90},{80,60}}, lineColor={255,200,0},
            fillColor={255,255,0}, fillPattern=FillPattern.Solid),
          Line(points={{65,95},{65,100}}, color={255,200,0}, thickness=1),
          Line(points={{85,75},{90,75}}, color={255,200,0}, thickness=1),
          Line(points={{45,75},{40,75}}, color={255,200,0}, thickness=1),
          Text(extent={{-80,-50},{80,-80}}, textColor={0,0,0},
            textString="Solar Thermal"),
          Text(extent={{-100,140},{100,110}}, textColor={0,0,255},
            textString="%name")}

          // Solar panel representation

          // Panel grid lines

          // Sun symbol

          // Text
),    Documentation(info="<html>
<h4>SimpleSolarThermal</h4>
<p>Simplified solar thermal collector model based on EN 12975 efficiency curve.</p>

<h5>Efficiency Model:</h5>
<pre>
η = η₀ - a₁*(Tm-Ta)/G - a₂*(Tm-Ta)²/G
</pre>

<h5>Default Parameters (Wolf CRK12 CPC):</h5>
<ul>
<li>η₀ = 0.642 (optical efficiency)</li>
<li>a₁ = 0.885 W/(m²·K)</li>
<li>a₂ = 0.001 W/(m²·K²)</li>
<li>AModule = 1.312 m²</li>
</ul>

<h5>Inputs:</h5>
<ul>
<li>STon: Pump on/off signal</li>
<li>qvRef: Reference volume flow [m³/s]</li>
<li>TReturn_degC: Return temperature [°C]</li>
<li>TAmbient_degC: Ambient temperature [°C]</li>
<li>GHI: Global Horizontal Irradiance [W/m²]</li>
</ul>

<h5>Outputs:</h5>
<ul>
<li>TFlow_degC: Flow temperature [°C]</li>
<li>TCollector_degC: Collector temperature [°C]</li>
<li>QSolarThermal: Thermal power [W]</li>
<li>qvActual: Actual volume flow [m³/s]</li>
</ul>
</html>"));
  end SimpleSolarThermal;
end HeatGenerators;
