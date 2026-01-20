within CoSES_Thermal_ProHMo_PHiL.Backup;
model TestBuilding_CHP_CB_DataDriven
  "Test model with realistic data from TRY Munich weather and MF5 occupancy profiles"

  // ============================================================
  // DATA FILE PATHS
  // Change these paths to match your local installation
  // ============================================================
  parameter String weatherFile = Modelica.Utilities.Files.loadResource(
    "modelica://CoSES_Thermal_ProHMo/Data/Munich_TRY_Weather.txt")
    "Path to weather data file";
  parameter String occupancyFile = Modelica.Utilities.Files.loadResource(
    "modelica://CoSES_Thermal_ProHMo/Data/MF5_Occupancy_Gains.txt")
    "Path to occupancy/gains data file";

  // ============================================================
  // SIMULATION PARAMETERS
  // ============================================================
  parameter Modelica.Units.SI.Time simDuration = 864000 "Simulation duration (10 days)";

  // ============================================================
  // CONTROL PARAMETERS (same as MF5 SimulationX model)
  // ============================================================
  parameter Modelica.Units.SI.Temperature T_CHP_on = 293.15 "CHP on when T_living < 20°C";
  parameter Modelica.Units.SI.Temperature T_CHP_off = 295.15 "CHP off when T_living > 22°C";
  parameter Modelica.Units.SI.Temperature T_CB_on = 291.15 "CB on when T_living < 18°C";
  parameter Modelica.Units.SI.Temperature T_CB_off = 293.15 "CB off when T_living > 20°C";
  parameter Real modulationGain = 0.2 "Modulation gain (5K error = 100%)";
  parameter Modelica.Units.SI.Temperature T_setpoint = 294.15 "Room temperature setpoint (21°C)";

  // ============================================================
  // DATA TABLES - Weather
  // ============================================================
  Modelica.Blocks.Sources.CombiTimeTable weatherData(
    tableOnFile=true,
    tableName="weather",
    fileName=weatherFile,
    columns={2,3,4},
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments)
    "Weather data: T_ambient[°C], GHI[W/m²], WindSpeed[m/s]"
    annotation(Placement(transformation(extent={{-180,60},{-160,80}})));

  // ============================================================
  // DATA TABLES - Occupancy and Internal Gains
  // ============================================================
  Modelica.Blocks.Sources.CombiTimeTable occupancyData(
    tableOnFile=true,
    tableName="occupancy",
    fileName=occupancyFile,
    columns={2,3,4,5,6,7,8},
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments)
    "Occupancy: nPers_liv, nPers_cel, nPers_roof, P_liv[W], P_cel[W], P_roof[W], Presence[%]"
    annotation(Placement(transformation(extent={{-180,20},{-160,40}})));

  // ============================================================
  // BUILDING MODEL (ThreeZoneBuilding_PHiL)
  // ============================================================
  // Replace with your actual ThreeZoneBuilding_PHiL instance
  // ThreeZoneBuilding_PHiL building(...);

  // ============================================================
  // SIMPLE CHP MODEL (SimpleCHP_v2)
  // ============================================================
  // SimpleCHP_v2 chp(
  //   QHeat_nominal = 12500,
  //   PElec_nominal = 5000,
  //   eta_thermal = 0.60,
  //   eta_electrical = 0.25);

  // ============================================================
  // SIMPLE CONDENSING BOILER MODEL (SimpleCondensingBoiler)
  // ============================================================
  // SimpleCondensingBoiler cb(
  //   QHeat_nominal = 50000,
  //   eta_nominal = 0.98,
  //   eta_noncondensing = 0.88);

  // ============================================================
  // CONTROL LOGIC
  // ============================================================

  // CHP Hysteresis Control
  Modelica.Blocks.Logical.Hysteresis CHP_hysteresis(
    uLow=T_CHP_on,
    uHigh=T_CHP_off)
    "CHP on when below T_CHP_on, off when above T_CHP_off"
    annotation(Placement(transformation(extent={{-60,80},{-40,100}})));

  Modelica.Blocks.Logical.Not CHP_control
    "Invert: heat when temp is LOW"
    annotation(Placement(transformation(extent={{-20,80},{0,100}})));

  // CB Hysteresis Control (backup)
  Modelica.Blocks.Logical.Hysteresis CB_hysteresis(
    uLow=T_CB_on,
    uHigh=T_CB_off)
    "CB on when below T_CB_on, off when above T_CB_off"
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
    uMin=0.0)
    "Limit modulation to 0-100%"
    annotation(Placement(transformation(extent={{-20,0},{0,20}})));

  // ============================================================
  // FLOW RATE (constant for this test)
  // ============================================================
  Modelica.Blocks.Sources.Constant flowRate(k=6)
    "Volume flow rate [L/min]"
    annotation(Placement(transformation(extent={{-180,-20},{-160,0}})));

  // ============================================================
  // INTERNAL VARIABLES FOR MONITORING
  // ============================================================
  Real T_ambient_degC "Outdoor temperature [°C]";
  Real T_living_degC "Living zone temperature [°C] (simulated)";
  Real T_cellar_degC "Cellar zone temperature [°C] (simulated)";
  Real T_roof_degC "Roof zone temperature [°C] (simulated)";
  Real nPersons_living "Number of people in living zone";
  Real nPersons_cellar "Number of people in cellar zone";
  Real nPersons_roof "Number of people in roof zone";
  Real P_app_living_W "Appliance power in living zone [W]";
  Real P_app_cellar_W "Appliance power in cellar zone [W]";
  Real P_app_roof_W "Appliance power in roof zone [W]";
  Real Modulation_pct "Modulation command [0-100%]";
  Boolean CHP_on "CHP on command";
  Boolean CB_on "CB on command";

  // Simulated zone temperatures (simple first-order response)
  // In real model, these come from ThreeZoneBuilding_PHiL
  Modelica.Units.SI.Temperature T_living_K(start=293.15, fixed=true);
  Modelica.Units.SI.Temperature T_cellar_K(start=288.15, fixed=true);
  Modelica.Units.SI.Temperature T_roof_K(start=291.15, fixed=true);

  // Thermal time constants [s]
  parameter Modelica.Units.SI.Time tau_living = 7200 "Living zone time constant";
  parameter Modelica.Units.SI.Time tau_cellar = 14400 "Cellar zone time constant";
  parameter Modelica.Units.SI.Time tau_roof = 5400 "Roof zone time constant";

  // Heat loss coefficients [W/K]
  parameter Real UA_living = 300 "Living zone UA value";
  parameter Real UA_cellar = 150 "Cellar zone UA value";
  parameter Real UA_roof = 200 "Roof zone UA value";

  // Thermal capacitance [J/K]
  parameter Real C_living = UA_living * tau_living;
  parameter Real C_cellar = UA_cellar * tau_cellar;
  parameter Real C_roof = UA_roof * tau_roof;

  // Heat gains
  Real Q_internal_living "Internal heat gains living [W]";
  Real Q_internal_cellar "Internal heat gains cellar [W]";
  Real Q_internal_roof "Internal heat gains roof [W]";
  Real Q_heating "Total heating power [W]";

equation
  // ============================================================
  // EXTRACT DATA FROM TABLES
  // ============================================================
  T_ambient_degC = weatherData.y[1];
  // GHI = weatherData.y[2];  // For future solar thermal
  // WindSpeed = weatherData.y[3];  // For future wind effects

  nPersons_living = occupancyData.y[1];
  nPersons_cellar = occupancyData.y[2];
  nPersons_roof = occupancyData.y[3];
  P_app_living_W = occupancyData.y[4];
  P_app_cellar_W = occupancyData.y[5];
  P_app_roof_W = occupancyData.y[6];
  // Presence_pct = occupancyData.y[7];

  // ============================================================
  // INTERNAL HEAT GAINS (80W per person + appliances)
  // ============================================================
  Q_internal_living = nPersons_living * 80 + P_app_living_W;
  Q_internal_cellar = nPersons_cellar * 80 + P_app_cellar_W;
  Q_internal_roof = nPersons_roof * 80 + P_app_roof_W;

  // ============================================================
  // HEATING POWER (simplified model)
  // In real model, this comes from CHP + CB outputs
  // ============================================================
  Q_heating = if CHP_on then
                (if CB_on then 12500 + 50000 * Modulation_pct/100
                 else 12500 * Modulation_pct/100)
              else 0;

  // ============================================================
  // SIMPLIFIED ZONE TEMPERATURE DYNAMICS
  // Replace with actual ThreeZoneBuilding_PHiL connections
  // ============================================================
  C_living * der(T_living_K) = Q_heating * 0.5
                               + Q_internal_living
                               - UA_living * (T_living_K - (T_ambient_degC + 273.15));
                                                // 50% to living zone

  C_cellar * der(T_cellar_K) = Q_heating * 0.2
                               + Q_internal_cellar
                               - UA_cellar * (T_cellar_K - (T_ambient_degC + 273.15 + 5));
                                                // 20% to cellar                            // Ground effect

  C_roof * der(T_roof_K) = Q_heating * 0.3
                           + Q_internal_roof
                           - UA_roof * (T_roof_K - (T_ambient_degC + 273.15));
                                            // 30% to roof

  // Convert to °C for outputs
  T_living_degC = T_living_K - 273.15;
  T_cellar_degC = T_cellar_K - 273.15;
  T_roof_degC = T_roof_K - 273.15;

  // ============================================================
  // CONTROL CONNECTIONS
  // ============================================================
  connect(T_living_K, CHP_hysteresis.u);
  connect(T_living_K, CB_hysteresis.u);
  connect(CHP_hysteresis.y, CHP_control.u);
  connect(CB_hysteresis.y, CB_control.u);

  CHP_on = CHP_control.y;
  CB_on = CB_control.y;

  connect(setpoint.y, tempError.u1);
  connect(T_living_K, tempError.u2);
  connect(tempError.y, modulationCalc.u);
  connect(modulationCalc.y, modLimiter.u);

  Modulation_pct = modLimiter.y * 100;

  annotation(
    experiment(
      StartTime=0,
      StopTime=864000,
      Tolerance=1e-06,
      Interval=60),
    Documentation(info="<html>
<h4>Test Model with Realistic Data</h4>
<p>This model uses CombiTimeTable to load:</p>
<ul>
<li><b>Munich_TRY_Weather.txt</b> - Munich TRY weather data (10 days)</li>
<li><b>MF5_Occupancy_Gains.txt</b> - Occupancy and appliance profiles</li>
</ul>

<h4>Data Sources</h4>
<p>Weather data based on DWD TRY (Test Reference Year) for Munich, Germany.</p>
<p>Occupancy patterns based on VDI 2078 / DIN EN 15251 German residential standards.</p>

<h4>Usage</h4>
<p>1. Place data files in your CoSES_Thermal_ProHMo/Data folder</p>
<p>2. Or modify weatherFile and occupancyFile paths</p>
<p>3. Connect to actual ThreeZoneBuilding_PHiL, SimpleCHP_v2, SimpleCondensingBoiler</p>
</html>"));
end TestBuilding_CHP_CB_DataDriven;
