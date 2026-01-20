within CoSES_Thermal_ProHMo_PHiL.Backup;
model TestBuilding_CHP_CB_DataDriven_v3_Enhanced
  "ENHANCED: Test model with additional plotting variables for thesis presentation"

  // ============================================================
  // SIMULATION PARAMETERS
  // ============================================================
  parameter Modelica.Units.SI.Time simDuration = 864000 "Simulation duration (10 days)";

  // ============================================================
  // CONTROL PARAMETERS - IMPROVED with wider hysteresis
  // ============================================================
  parameter Modelica.Units.SI.Temperature T_CHP_on = 292.15 "CHP on when T_living < 19°C";
  parameter Modelica.Units.SI.Temperature T_CHP_off = 295.15 "CHP off when T_living > 22°C";
  parameter Modelica.Units.SI.Temperature T_CB_on = 289.15 "CB on when T_living < 16°C (emergency)";
  parameter Modelica.Units.SI.Temperature T_CB_off = 294.15 "CB off when T_living > 21°C";
  parameter Real modulationGain = 0.15 "Modulation gain (6.7K error = 100%)";
  parameter Modelica.Units.SI.Temperature T_setpoint = 294.15 "Room temperature setpoint (21°C)";

  // CHP and CB nominal powers
  parameter Modelica.Units.SI.Power Q_CHP_nom = 12500 "CHP nominal thermal power [W]";
  parameter Modelica.Units.SI.Power Q_CB_nom = 50000 "CB nominal thermal power [W]";

  // Electrical parameters for CHP
  parameter Modelica.Units.SI.Power P_elec_nom = 5000 "CHP nominal electrical power [W]";

  // ============================================================
  // DATA TABLES - Weather (INLINE - Munich TRY Winter 10 days)
  // ============================================================
  Modelica.Blocks.Sources.CombiTimeTable weatherData(
    tableOnFile=false,
    table=[
      0, -3.0, 0, 2.5;
      3600, -4.2, 0, 2.8;
      7200, -5.0, 0, 3.0;
      10800, -5.5, 0, 2.2;
      14400, -5.2, 0, 1.8;
      18000, -4.0, 10, 1.5;
      21600, -2.5, 45, 1.2;
      25200, -1.0, 120, 1.5;
      28800, 0.5, 180, 2.0;
      32400, 1.8, 210, 2.2;
      36000, 2.5, 190, 2.5;
      39600, 2.8, 150, 2.8;
      43200, 2.5, 100, 2.5;
      46800, 1.5, 50, 2.2;
      50400, 0.5, 10, 2.0;
      54000, -0.5, 0, 1.8;
      57600, -1.5, 0, 2.0;
      61200, -2.5, 0, 2.5;
      64800, -3.2, 0, 2.8;
      68400, -3.8, 0, 3.0;
      72000, -4.2, 0, 2.5;
      75600, -4.5, 0, 2.2;
      79200, -4.8, 0, 2.0;
      82800, -5.0, 0, 1.8;
      86400, -5.2, 0, 2.0;
      172800, -4.5, 0, 1.2;
      259200, -2.0, 0, 0.8;
      345600, -1.0, 0, 1.0;
      432000, -9.0, 0, 2.5;
      518400, -9.5, 0, 1.5;
      604800, -4.5, 0, 1.0;
      691200, -2.0, 0, 0.8;
      777600, -1.0, 0, 1.0;
      864000, 0.0, 0, 1.5],
    columns={2,3,4},
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments)
    "Weather: T_ambient[°C], Solar[W/m2], Wind[m/s]"
    annotation(Placement(transformation(extent={{-180,60},{-160,80}})));

  // ============================================================
  // DATA TABLES - Occupancy (simplified daily pattern)
  // ============================================================
  Modelica.Blocks.Sources.CombiTimeTable occupancyData(
    tableOnFile=false,
    table=[
      0, 2, 0, 0, 100, 50, 30;
      21600, 2, 0, 0, 150, 50, 30;
      28800, 0, 0, 0, 50, 30, 20;
      64800, 2, 0, 0, 200, 50, 30;
      86400, 2, 0, 0, 100, 50, 30],
    columns={2,3,4,5,6,7},
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments)
    "Occupancy: nPers_liv, nPers_cel, nPers_roof, P_liv[W], P_cel[W], P_roof[W]"
    annotation(Placement(transformation(extent={{-180,20},{-160,40}})));

  // ============================================================
  // CONTROL LOGIC
  // ============================================================
  Modelica.Blocks.Logical.Hysteresis CHP_hysteresis(
    uLow=T_CHP_on,
    uHigh=T_CHP_off)
    "CHP on when below 19°C, off when above 22°C"
    annotation(Placement(transformation(extent={{-60,80},{-40,100}})));

  Modelica.Blocks.Logical.Not CHP_control
    "Invert: heat when temp is LOW"
    annotation(Placement(transformation(extent={{-20,80},{0,100}})));

  Modelica.Blocks.Logical.Hysteresis CB_hysteresis(
    uLow=T_CB_on,
    uHigh=T_CB_off)
    "CB on when below 16°C, off when above 21°C (emergency backup)"
    annotation(Placement(transformation(extent={{-60,40},{-40,60}})));

  Modelica.Blocks.Logical.Not CB_control
    "Invert: heat when temp is LOW"
    annotation(Placement(transformation(extent={{-20,40},{0,60}})));

  Modelica.Blocks.Math.Feedback tempError
    "Setpoint - Actual temperature"
    annotation(Placement(transformation(extent={{-100,0},{-80,20}})));

  Modelica.Blocks.Sources.Constant setpoint(k=T_setpoint)
    "Room temperature setpoint"
    annotation(Placement(transformation(extent={{-140,0},{-120,20}})));

  Modelica.Blocks.Math.Gain modulationCalc(k=modulationGain)
    "Convert temperature error to modulation"
    annotation(Placement(transformation(extent={{-60,0},{-40,20}})));

  Modelica.Blocks.Nonlinear.Limiter modLimiter(
    uMax=1.0,
    uMin=0.2)
    "Limit modulation to 20-100% (minimum firing rate)"
    annotation(Placement(transformation(extent={{-20,0},{0,20}})));

  // ============================================================
  // PRIMARY MONITORING VARIABLES
  // ============================================================
  Real T_ambient_degC "Outdoor temperature [°C]";
  Real T_living_degC "Living zone temperature [°C]";
  Real T_cellar_degC "Cellar zone temperature [°C]";
  Real T_roof_degC "Roof zone temperature [°C]";
  Real nPersons_living "Number of people in living zone";
  Real nPersons_cellar "Number of people in cellar zone";
  Real nPersons_roof "Number of people in roof zone";
  Real P_app_living_W "Appliance power in living zone [W]";
  Real P_app_cellar_W "Appliance power in cellar zone [W]";
  Real P_app_roof_W "Appliance power in roof zone [W]";
  Real Modulation_pct "Modulation command [0-100%]";
  Boolean CHP_on "CHP on command";
  Boolean CB_on "CB on command";

  // Simulated zone temperatures
  Modelica.Units.SI.Temperature T_living_K(start=293.15, fixed=true);
  Modelica.Units.SI.Temperature T_cellar_K(start=288.15, fixed=true);
  Modelica.Units.SI.Temperature T_roof_K(start=291.15, fixed=true);

  // Thermal parameters
  parameter Modelica.Units.SI.Time tau_living = 7200 "Living zone time constant";
  parameter Modelica.Units.SI.Time tau_cellar = 14400 "Cellar zone time constant";
  parameter Modelica.Units.SI.Time tau_roof = 7200 "Roof zone time constant";

  parameter Real UA_living = 300 "Living zone UA value [W/K]";
  parameter Real UA_cellar = 150 "Cellar zone UA value [W/K]";
  parameter Real UA_roof = 180 "Roof zone UA value [W/K]";

  parameter Real C_living = UA_living * tau_living;
  parameter Real C_cellar = UA_cellar * tau_cellar;
  parameter Real C_roof = UA_roof * tau_roof;

  // ============================================================
  // HEAT FLOW VARIABLES
  // ============================================================
  Real Q_internal_living "Internal heat gains living [W]";
  Real Q_internal_cellar "Internal heat gains cellar [W]";
  Real Q_internal_roof "Internal heat gains roof [W]";
  Real Q_CHP "CHP thermal power [W]";
  Real Q_CB "CB thermal power [W]";
  Real Q_heating "Total heating power [W]";

  // ============================================================
  // NEW: ENHANCED PLOTTING VARIABLES (Based on Thesis Papers)
  // ============================================================

  // Heat losses through building envelope
  Real Q_loss_living "Heat loss from living zone [W]";
  Real Q_loss_cellar "Heat loss from cellar zone [W]";
  Real Q_loss_roof "Heat loss from roof zone [W]";
  Real Q_loss_total "Total building heat loss [W]";

  // In kW for easier plotting
  Real Q_CHP_kW "CHP thermal power [kW]";
  Real Q_CB_kW "CB thermal power [kW]";
  Real Q_heating_kW "Total heating power [kW]";
  Real Q_loss_total_kW "Total heat loss [kW]";

  // Electrical output (CHP)
  Real P_elec_CHP "CHP electrical power [W]";
  Real P_elec_CHP_kW "CHP electrical power [kW]";

  // Efficiency metrics (inspired by Zinsmeister et al.)
  Real COP_system "System coefficient of performance [-]";
  Real CHP_runtime_fraction "CHP runtime fraction [-]";
  Real CB_runtime_fraction "CB runtime fraction [-]";

  // Temperature differences for analysis
  Real deltaT_living_ambient "Living zone - Ambient temperature difference [K]";
  Real deltaT_supply_return "Approximate supply-return temperature difference [K]";

  // Heat balance check
  Real Q_balance "Heat balance: input - loss - storage [W]";

  // Time-based metrics
  Real time_hours "Simulation time [hours]";
  Real time_days "Simulation time [days]";

  // ============================================================
  // ENERGY MONITORING (Cumulative)
  // ============================================================
  Real E_CHP(start=0, fixed=true) "CHP thermal energy delivered [J]";
  Real E_CB(start=0, fixed=true) "CB thermal energy delivered [J]";
  Real E_total(start=0, fixed=true) "Total heating energy [J]";
  Real E_elec_CHP(start=0, fixed=true) "CHP electrical energy generated [J]";
  Real E_loss(start=0, fixed=true) "Total heat loss energy [J]";

  // In kWh for easier interpretation
  Real E_CHP_kWh "CHP thermal energy [kWh]";
  Real E_CB_kWh "CB thermal energy [kWh]";
  Real E_total_kWh "Total heating energy [kWh]";
  Real E_elec_CHP_kWh "CHP electrical energy [kWh]";
  Real E_loss_kWh "Total heat loss energy [kWh]";

  // ============================================================
  // COMFORT MONITORING
  // ============================================================
  Real comfort_violation_time(start=0, fixed=true) "Time outside comfort band [s]";
  Boolean in_comfort_band "True if T_living in 18-24°C";
  Real comfort_violation_hours "Time outside comfort band [hours]";
  Real comfort_percentage "Percentage of time in comfort [%]";

  // Runtime tracking
  Real CHP_runtime(start=0, fixed=true) "CHP total runtime [s]";
  Real CB_runtime(start=0, fixed=true) "CB total runtime [s]";

equation
  // ============================================================
  // EXTRACT DATA FROM TABLES
  // ============================================================
  T_ambient_degC = weatherData.y[1];

  nPersons_living = occupancyData.y[1];
  nPersons_cellar = occupancyData.y[2];
  nPersons_roof = occupancyData.y[3];
  P_app_living_W = occupancyData.y[4];
  P_app_cellar_W = occupancyData.y[5];
  P_app_roof_W = occupancyData.y[6];

  // ============================================================
  // INTERNAL HEAT GAINS (80W per person + appliances)
  // ============================================================
  Q_internal_living = nPersons_living * 80 + P_app_living_W;
  Q_internal_cellar = nPersons_cellar * 80 + P_app_cellar_W;
  Q_internal_roof = nPersons_roof * 80 + P_app_roof_W;

  // ============================================================
  // HEATING POWER
  // ============================================================
  Q_CHP = if CHP_on then Q_CHP_nom * Modulation_pct/100 else 0;
  Q_CB = if CB_on then Q_CB_nom * Modulation_pct/100 else 0;
  Q_heating = Q_CHP + Q_CB;

  // CHP electrical output (proportional to thermal)
  P_elec_CHP = if CHP_on then P_elec_nom * Modulation_pct/100 else 0;

  // ============================================================
  // HEAT LOSSES
  // ============================================================
  Q_loss_living = UA_living * (T_living_K - (T_ambient_degC + 273.15));
  Q_loss_cellar = UA_cellar * (T_cellar_K - (T_ambient_degC + 273.15 + 5));
  Q_loss_roof = UA_roof * (T_roof_K - (T_ambient_degC + 273.15));
  Q_loss_total = Q_loss_living + Q_loss_cellar + Q_loss_roof;

  // ============================================================
  // ZONE TEMPERATURE DYNAMICS
  // ============================================================
  C_living * der(T_living_K) = Q_heating * 0.5
                               + Q_internal_living
                               - Q_loss_living;

  C_cellar * der(T_cellar_K) = Q_heating * 0.2
                               + Q_internal_cellar
                               - Q_loss_cellar;

  C_roof * der(T_roof_K) = Q_heating * 0.3
                           + Q_internal_roof
                           - Q_loss_roof;

  // Convert to °C
  T_living_degC = T_living_K - 273.15;
  T_cellar_degC = T_cellar_K - 273.15;
  T_roof_degC = T_roof_K - 273.15;

  // ============================================================
  // CONTROL CONNECTIONS
  // ============================================================
  CHP_hysteresis.u = T_living_K;
  CB_hysteresis.u = T_living_K;

  connect(CHP_hysteresis.y, CHP_control.u);
  connect(CB_hysteresis.y, CB_control.u);

  CHP_on = CHP_control.y;
  CB_on = CB_control.y;

  connect(setpoint.y, tempError.u1);
  tempError.u2 = T_living_K;
  connect(tempError.y, modulationCalc.u);
  connect(modulationCalc.y, modLimiter.u);

  Modulation_pct = modLimiter.y * 100;

  // ============================================================
  // DERIVED PLOTTING VARIABLES
  // ============================================================

  // kW conversions
  Q_CHP_kW = Q_CHP / 1000;
  Q_CB_kW = Q_CB / 1000;
  Q_heating_kW = Q_heating / 1000;
  Q_loss_total_kW = Q_loss_total / 1000;
  P_elec_CHP_kW = P_elec_CHP / 1000;

  // Temperature differences
  deltaT_living_ambient = T_living_K - (T_ambient_degC + 273.15);
  deltaT_supply_return = if Q_heating > 100 then 10 else 0; // Simplified

  // Heat balance (should be ~0 in steady state)
  Q_balance = Q_heating + Q_internal_living + Q_internal_cellar + Q_internal_roof
              - Q_loss_total;

  // Time conversions
  time_hours = time / 3600;
  time_days = time / 86400;

  // System COP (simplified - ratio of heat delivered to "equivalent" input)
  COP_system = if Q_heating > 100 then Q_heating / (Q_heating * 0.9 + P_elec_CHP * 2.5) else 0;

  // ============================================================
  // ENERGY INTEGRATION
  // ============================================================
  der(E_CHP) = Q_CHP;
  der(E_CB) = Q_CB;
  der(E_total) = Q_heating;
  der(E_elec_CHP) = P_elec_CHP;
  der(E_loss) = Q_loss_total;

  // kWh conversions (divide by 3.6e6 to convert J to kWh)
  E_CHP_kWh = E_CHP / 3.6e6;
  E_CB_kWh = E_CB / 3.6e6;
  E_total_kWh = E_total / 3.6e6;
  E_elec_CHP_kWh = E_elec_CHP / 3.6e6;
  E_loss_kWh = E_loss / 3.6e6;

  // ============================================================
  // COMFORT MONITORING
  // ============================================================
  in_comfort_band = T_living_K >= 291.15 and T_living_K <= 297.15;  // 18-24°C
  der(comfort_violation_time) = if in_comfort_band then 0 else 1;
  comfort_violation_hours = comfort_violation_time / 3600;
  comfort_percentage = if time > 0 then (1 - comfort_violation_time/time) * 100 else 100;

  // Runtime tracking
  der(CHP_runtime) = if CHP_on then 1 else 0;
  der(CB_runtime) = if CB_on then 1 else 0;
  CHP_runtime_fraction = if time > 0 then CHP_runtime / time else 0;
  CB_runtime_fraction = if time > 0 then CB_runtime / time else 0;

  annotation(
    experiment(
      StartTime=0,
      StopTime=864000,
      Tolerance=1e-06,
      Interval=60),
    Documentation(info="<html>
<h4>ENHANCED Test Model v3 - For Thesis Presentation</h4>

<h5>New Variables for Plotting (Based on Supervisor's Thesis Papers):</h5>

<h6>1. Heat Balance Analysis:</h6>
<ul>
<li>Q_loss_living, Q_loss_cellar, Q_loss_roof, Q_loss_total - Heat losses [W]</li>
<li>Q_balance - Heat balance check (should be ~0) [W]</li>
</ul>

<h6>2. kW/kWh Conversions (Easier to read):</h6>
<ul>
<li>Q_CHP_kW, Q_CB_kW, Q_heating_kW, Q_loss_total_kW - Powers [kW]</li>
<li>E_CHP_kWh, E_CB_kWh, E_total_kWh, E_loss_kWh - Energies [kWh]</li>
</ul>

<h6>3. CHP Electrical Output:</h6>
<ul>
<li>P_elec_CHP, P_elec_CHP_kW - Electrical power [W, kW]</li>
<li>E_elec_CHP_kWh - Cumulative electrical energy [kWh]</li>
</ul>

<h6>4. Efficiency Metrics:</h6>
<ul>
<li>COP_system - System coefficient of performance [-]</li>
<li>CHP_runtime_fraction, CB_runtime_fraction - Runtime fractions [-]</li>
</ul>

<h6>5. Comfort Analysis:</h6>
<ul>
<li>comfort_percentage - Time in comfort band [%]</li>
<li>comfort_violation_hours - Time outside comfort [hours]</li>
</ul>

<h6>6. Time Conversions:</h6>
<ul>
<li>time_hours, time_days - For easier x-axis labeling</li>
</ul>

<h5>Recommended Plots for Thesis Presentation:</h5>
<ol>
<li><b>Temperature Overview:</b> T_living_degC, T_cellar_degC, T_roof_degC, T_ambient_degC</li>
<li><b>Heat Generator Comparison:</b> Q_CHP_kW, Q_CB_kW vs time_days</li>
<li><b>Energy Delivery:</b> E_CHP_kWh, E_CB_kWh, E_total_kWh vs time_days</li>
<li><b>Heat Balance:</b> Q_heating_kW, Q_loss_total_kW vs time_days</li>
<li><b>CHP Performance:</b> Q_CHP_kW, P_elec_CHP_kW vs time_days</li>
<li><b>Control Behavior:</b> CHP_on, CB_on, Modulation_pct vs T_living_degC</li>
<li><b>Runtime Analysis:</b> CHP_runtime_fraction, CB_runtime_fraction (final values)</li>
<li><b>Comfort Analysis:</b> comfort_percentage (final value)</li>
</ol>
</html>"));
end TestBuilding_CHP_CB_DataDriven_v3_Enhanced;
