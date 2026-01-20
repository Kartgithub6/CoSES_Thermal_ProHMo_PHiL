within CoSES_Thermal_ProHMo_PHiL.Backup;
model TestBuilding_CHP_CB_DataDriven_v2
  "IMPROVED: Test model with anti-chattering control and corrected heating logic"

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
      90000, -5.5, 0, 2.2;
      93600, -5.8, 0, 2.5;
      97200, -6.0, 0, 2.8;
      100800, -5.5, 0, 2.5;
      104400, -4.5, 15, 2.0;
      108000, -3.0, 60, 1.8;
      111600, -1.5, 150, 1.5;
      115200, 0.0, 200, 1.8;
      118800, 1.2, 220, 2.0;
      122400, 2.0, 200, 2.2;
      126000, 2.2, 160, 2.5;
      129600, 1.8, 100, 2.2;
      133200, 1.0, 40, 2.0;
      136800, 0.0, 5, 1.8;
      140400, -1.0, 0, 2.0;
      144000, -2.0, 0, 2.2;
      147600, -3.0, 0, 2.5;
      151200, -3.5, 0, 2.8;
      154800, -4.0, 0, 2.5;
      158400, -4.2, 0, 2.2;
      162000, -4.5, 0, 2.0;
      165600, -4.8, 0, 1.8;
      169200, -5.0, 0, 1.5;
      172800, -4.5, 0, 1.2;
      176400, -4.0, 0, 1.5;
      180000, -3.5, 0, 1.8;
      183600, -3.0, 0, 2.0;
      187200, -2.5, 0, 1.8;
      190800, -1.5, 20, 1.5;
      194400, 0.0, 80, 1.2;
      198000, 1.5, 160, 1.5;
      201600, 3.0, 210, 1.8;
      205200, 4.5, 240, 2.0;
      208800, 5.5, 220, 2.2;
      212400, 5.8, 180, 2.0;
      216000, 5.5, 120, 1.8;
      219600, 4.5, 60, 1.5;
      223200, 3.5, 15, 1.2;
      226800, 2.5, 0, 1.5;
      230400, 1.5, 0, 1.8;
      234000, 0.5, 0, 2.0;
      237600, 0.0, 0, 2.2;
      241200, -0.5, 0, 2.0;
      244800, -1.0, 0, 1.8;
      248400, -1.5, 0, 1.5;
      252000, -2.0, 0, 1.2;
      255600, -2.5, 0, 1.0;
      259200, -2.0, 0, 0.8;
      262800, -1.5, 0, 1.0;
      266400, -1.0, 0, 1.2;
      270000, -0.5, 0, 1.5;
      273600, 0.0, 5, 1.5;
      277200, 1.0, 30, 1.2;
      280800, 2.5, 100, 1.0;
      284400, 4.0, 180, 1.2;
      288000, 5.5, 230, 1.5;
      291600, 6.5, 250, 1.8;
      295200, 7.2, 240, 2.0;
      298800, 7.5, 200, 1.8;
      302400, 7.2, 140, 1.5;
      306000, 6.5, 80, 1.2;
      309600, 5.5, 25, 1.0;
      313200, 4.5, 5, 1.2;
      316800, 3.5, 0, 1.5;
      320400, 2.5, 0, 1.8;
      324000, 2.0, 0, 2.0;
      327600, 1.5, 0, 1.8;
      331200, 1.0, 0, 1.5;
      334800, 0.5, 0, 1.2;
      338400, 0.0, 0, 1.0;
      342000, -0.5, 0, 0.8;
      345600, -1.0, 0, 1.0;
      349200, -2.0, 0, 1.5;
      352800, -3.0, 0, 2.0;
      356400, -4.0, 0, 2.5;
      360000, -5.0, 0, 3.0;
      363600, -5.5, 5, 3.2;
      367200, -5.0, 40, 2.8;
      370800, -4.0, 100, 2.5;
      374400, -3.0, 150, 2.8;
      378000, -2.0, 170, 3.0;
      381600, -1.5, 160, 3.2;
      385200, -1.8, 130, 3.0;
      388800, -2.2, 80, 2.8;
      392400, -2.8, 40, 2.5;
      396000, -3.5, 10, 2.8;
      399600, -4.5, 0, 3.0;
      403200, -5.5, 0, 3.2;
      406800, -6.5, 0, 3.5;
      410400, -7.0, 0, 3.8;
      414000, -7.5, 0, 4.0;
      417600, -8.0, 0, 3.5;
      421200, -8.2, 0, 3.2;
      424800, -8.5, 0, 3.0;
      428400, -8.8, 0, 2.8;
      432000, -9.0, 0, 2.5;
      435600, -9.2, 0, 2.8;
      439200, -9.5, 0, 3.0;
      442800, -9.8, 0, 3.2;
      446400, -9.5, 0, 3.0;
      450000, -8.5, 10, 2.8;
      453600, -7.0, 50, 2.5;
      457200, -5.5, 120, 2.2;
      460800, -4.0, 180, 2.5;
      464400, -3.0, 200, 2.8;
      468000, -2.5, 190, 3.0;
      471600, -2.8, 150, 2.8;
      475200, -3.5, 100, 2.5;
      478800, -4.5, 50, 2.2;
      482400, -5.5, 15, 2.5;
      486000, -6.5, 0, 2.8;
      489600, -7.5, 0, 3.0;
      493200, -8.0, 0, 3.2;
      496800, -8.5, 0, 3.0;
      500400, -8.8, 0, 2.8;
      504000, -9.0, 0, 2.5;
      507600, -9.2, 0, 2.2;
      511200, -9.5, 0, 2.0;
      514800, -9.8, 0, 1.8;
      518400, -9.5, 0, 1.5;
      522000, -9.0, 0, 1.8;
      525600, -8.5, 0, 2.0;
      529200, -8.0, 0, 2.2;
      532800, -7.5, 0, 2.0;
      536400, -6.5, 15, 1.8;
      540000, -5.0, 60, 1.5;
      543600, -3.5, 140, 1.8;
      547200, -2.0, 200, 2.0;
      550800, -0.5, 230, 2.2;
      554400, 0.5, 220, 2.0;
      558000, 1.0, 180, 1.8;
      561600, 0.8, 120, 1.5;
      565200, 0.2, 60, 1.2;
      568800, -0.5, 20, 1.5;
      572400, -1.5, 5, 1.8;
      576000, -2.5, 0, 2.0;
      579600, -3.0, 0, 2.2;
      583200, -3.5, 0, 2.0;
      586800, -4.0, 0, 1.8;
      590400, -4.2, 0, 1.5;
      594000, -4.5, 0, 1.2;
      597600, -4.8, 0, 1.0;
      601200, -5.0, 0, 0.8;
      604800, -4.5, 0, 1.0;
      608400, -4.0, 0, 1.2;
      612000, -3.5, 0, 1.5;
      615600, -3.0, 0, 1.8;
      619200, -2.5, 5, 1.5;
      622800, -1.5, 25, 1.2;
      626400, 0.0, 90, 1.0;
      630000, 1.5, 170, 1.2;
      633600, 3.0, 220, 1.5;
      637200, 4.5, 250, 1.8;
      640800, 5.5, 240, 2.0;
      644400, 6.0, 200, 1.8;
      648000, 5.8, 140, 1.5;
      651600, 5.0, 80, 1.2;
      655200, 4.0, 30, 1.0;
      658800, 3.0, 5, 1.2;
      662400, 2.0, 0, 1.5;
      666000, 1.0, 0, 1.8;
      669600, 0.5, 0, 2.0;
      673200, 0.0, 0, 1.8;
      676800, -0.5, 0, 1.5;
      680400, -1.0, 0, 1.2;
      684000, -1.5, 0, 1.0;
      687600, -2.0, 0, 0.8;
      691200, -1.5, 0, 1.0;
      694800, -1.0, 0, 1.2;
      698400, -0.5, 0, 1.5;
      702000, 0.0, 0, 1.8;
      705600, 0.5, 10, 1.5;
      709200, 1.5, 40, 1.2;
      712800, 3.0, 80, 1.0;
      716400, 4.5, 120, 1.2;
      720000, 5.5, 140, 1.5;
      723600, 6.2, 130, 1.8;
      727200, 6.5, 110, 2.0;
      730800, 6.2, 90, 1.8;
      734400, 5.8, 60, 1.5;
      738000, 5.0, 30, 1.2;
      741600, 4.0, 10, 1.0;
      745200, 3.0, 0, 1.2;
      748800, 2.0, 0, 1.5;
      752400, 1.5, 0, 1.8;
      756000, 1.0, 0, 2.0;
      759600, 0.5, 0, 1.8;
      763200, 0.0, 0, 1.5;
      766800, -0.5, 0, 1.2;
      770400, -1.0, 0, 1.0;
      774000, -1.5, 0, 0.8;
      777600, -2.0, 0, 1.0;
      781200, -2.5, 0, 1.5;
      784800, -3.0, 0, 2.0;
      788400, -3.5, 0, 2.5;
      792000, -4.0, 0, 2.8;
      795600, -3.5, 10, 2.5;
      799200, -2.5, 50, 2.2;
      802800, -1.5, 120, 2.0;
      806400, -0.5, 180, 2.2;
      810000, 0.5, 200, 2.5;
      813600, 1.0, 190, 2.8;
      817200, 1.2, 160, 2.5;
      820800, 0.8, 100, 2.2;
      824400, 0.0, 50, 2.0;
      828000, -0.8, 15, 2.2;
      831600, -1.5, 0, 2.5;
      835200, -2.5, 0, 2.8;
      838800, -3.0, 0, 3.0;
      842400, -3.5, 0, 2.8;
      846000, -4.0, 0, 2.5;
      849600, -4.5, 0, 2.2;
      853200, -5.0, 0, 2.0;
      856800, -5.5, 0, 1.8;
      860400, -6.0, 0, 1.5;
      864000, -6.0, 0, 1.5],
    columns={2,3,4},
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments)
    "Weather data: T_ambient[°C], GHI[W/m²], WindSpeed[m/s]"
    annotation(Placement(transformation(extent={{-180,60},{-160,80}})));

  // ============================================================
  // DATA TABLES - Occupancy (INLINE)
  // ============================================================
  Modelica.Blocks.Sources.CombiTimeTable occupancyData(
    tableOnFile=false,
    table=[
      0, 0, 0, 0, 30, 20, 10;
      21600, 2, 0, 0, 150, 20, 50;
      25200, 3, 0, 1, 300, 20, 150;
      28800, 1, 0, 1, 100, 20, 200;
      32400, 0, 0, 1, 80, 20, 250;
      43200, 0, 0, 1, 150, 20, 200;
      57600, 0, 0, 1, 80, 20, 200;
      61200, 2, 0, 1, 200, 20, 150;
      64800, 3, 0, 1, 400, 20, 100;
      68400, 4, 0, 0, 500, 20, 50;
      72000, 4, 0, 0, 350, 50, 30;
      75600, 3, 1, 0, 300, 80, 20;
      79200, 2, 1, 0, 250, 80, 20;
      82800, 0, 0, 0, 50, 20, 10;
      86400, 0, 0, 0, 30, 20, 10],
    columns={2,3,4,5,6,7},
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments)
    "Occupancy: nPers_liv, nPers_cel, nPers_roof, P_liv[W], P_cel[W], P_roof[W]"
    annotation(Placement(transformation(extent={{-180,20},{-160,40}})));

  // ============================================================
  // CONTROL LOGIC - IMPROVED
  // ============================================================

  // CHP Hysteresis Control (wider band)
  Modelica.Blocks.Logical.Hysteresis CHP_hysteresis(
    uLow=T_CHP_on,
    uHigh=T_CHP_off)
    "CHP on when below 19°C, off when above 22°C"
    annotation(Placement(transformation(extent={{-60,80},{-40,100}})));

  Modelica.Blocks.Logical.Not CHP_control
    "Invert: heat when temp is LOW"
    annotation(Placement(transformation(extent={{-20,80},{0,100}})));

  // CB Hysteresis Control (wider band - backup only)
  Modelica.Blocks.Logical.Hysteresis CB_hysteresis(
    uLow=T_CB_on,
    uHigh=T_CB_off)
    "CB on when below 16°C, off when above 21°C (emergency backup)"
    annotation(Placement(transformation(extent={{-60,40},{-40,60}})));

  Modelica.Blocks.Logical.Not CB_control
    "Invert: heat when temp is LOW"
    annotation(Placement(transformation(extent={{-20,40},{0,60}})));

  // Temperature Error for Modulation
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
  // MONITORING VARIABLES
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

  // Thermal parameters - IMPROVED for roof zone
  parameter Modelica.Units.SI.Time tau_living = 7200 "Living zone time constant";
  parameter Modelica.Units.SI.Time tau_cellar = 14400 "Cellar zone time constant";
  parameter Modelica.Units.SI.Time tau_roof = 7200 "Roof zone time constant (increased)";

  parameter Real UA_living = 300 "Living zone UA value [W/K]";
  parameter Real UA_cellar = 150 "Cellar zone UA value [W/K]";
  parameter Real UA_roof = 180 "Roof zone UA value [W/K] (reduced losses)";

  parameter Real C_living = UA_living * tau_living;
  parameter Real C_cellar = UA_cellar * tau_cellar;
  parameter Real C_roof = UA_roof * tau_roof;

  // Heat flow variables
  Real Q_internal_living "Internal heat gains living [W]";
  Real Q_internal_cellar "Internal heat gains cellar [W]";
  Real Q_internal_roof "Internal heat gains roof [W]";
  Real Q_CHP "CHP thermal power [W]";
  Real Q_CB "CB thermal power [W]";
  Real Q_heating "Total heating power [W]";

  // Energy monitoring
  Real E_CHP(start=0, fixed=true) "CHP energy delivered [J]";
  Real E_CB(start=0, fixed=true) "CB energy delivered [J]";
  Real E_total(start=0, fixed=true) "Total heating energy [J]";

  // Comfort monitoring
  Real comfort_violation_time(start=0, fixed=true) "Time outside comfort band [s]";
  Boolean in_comfort_band "True if T_living in 18-24°C";

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
  // HEATING POWER - CORRECTED LOGIC
  // ============================================================
  // CHP power: modulated based on demand
  Q_CHP = if CHP_on then Q_CHP_nom * Modulation_pct/100 else 0;

  // CB power: only when needed (emergency backup), also modulated
  Q_CB = if CB_on then Q_CB_nom * Modulation_pct/100 else 0;

  // Total heating
  Q_heating = Q_CHP + Q_CB;

  // ============================================================
  // ZONE TEMPERATURE DYNAMICS
  // ============================================================
  C_living * der(T_living_K) = Q_heating * 0.5
                               + Q_internal_living
                               - UA_living * (T_living_K - (T_ambient_degC + 273.15));

  C_cellar * der(T_cellar_K) = Q_heating * 0.2
                               + Q_internal_cellar
                               - UA_cellar * (T_cellar_K - (T_ambient_degC + 273.15 + 5));

  C_roof * der(T_roof_K) = Q_heating * 0.3
                           + Q_internal_roof
                           - UA_roof * (T_roof_K - (T_ambient_degC + 273.15));

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
  // ENERGY INTEGRATION
  // ============================================================
  der(E_CHP) = Q_CHP;
  der(E_CB) = Q_CB;
  der(E_total) = Q_heating;

  // ============================================================
  // COMFORT MONITORING
  // ============================================================
  in_comfort_band = T_living_K >= 291.15 and T_living_K <= 297.15;  // 18-24°C
  der(comfort_violation_time) = if in_comfort_band then 0 else 1;

  annotation(
    experiment(
      StartTime=0,
      StopTime=864000,
      Tolerance=1e-06,
      Interval=60),
    Documentation(info="<html>
<h4>IMPROVED Test Model v2</h4>
<p>Changes from v1:</p>
<ul>
<li>Wider hysteresis bands to prevent chattering</li>
<li>Corrected heating power calculation</li>
<li>Minimum modulation rate (20%)</li>
<li>Improved roof zone thermal parameters</li>
<li>Added energy integration variables</li>
<li>Added comfort monitoring</li>
</ul>

<h4>New Variables to Plot</h4>
<ul>
<li>Q_CHP, Q_CB - Individual heat generator powers</li>
<li>E_CHP, E_CB, E_total - Cumulative energy [J]</li>
<li>comfort_violation_time - Time outside comfort band [s]</li>
<li>in_comfort_band - Comfort status</li>
</ul>

<h4>Expected Results</h4>
<ul>
<li>CB should rarely activate (only during extreme cold)</li>
<li>CHP provides base load with modulation</li>
<li>Modulation varies with heating demand</li>
</ul>
</html>"));
end TestBuilding_CHP_CB_DataDriven_v2;
