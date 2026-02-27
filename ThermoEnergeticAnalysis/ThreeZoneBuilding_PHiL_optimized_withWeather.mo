within CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis;
model ThreeZoneBuilding_PHiL_optimized_withWeather
  "PHiL wrapper with weather, building types, annotations, and weather file dropdown - COMPLETE"

  replaceable package Medium = Modelica.Media.Water.StandardWater;

  // ═══════════════════════════════════════════════════════════════════
  // 🏠 BUILDING TYPE PARAMETER
  // ═══════════════════════════════════════════════════════════════════
  replaceable parameter CoSES_Thermal_ProHMo_PHiL.Data.SmallHouse
                                                                buildingData
    constrainedby CoSES_Thermal_ProHMo_PHiL.Data.BaseBuilding
    "Building type - SELECT from available building types"
    annotation(
      choicesAllMatching=true,
      Dialog(group="Building Configuration", tab="General"));

  // ═══════════════════════════════════════════════════════════════════
  // 🌤️ WEATHER CONFIGURATION WITH DROPDOWN
  // ═══════════════════════════════════════════════════════════════════
  parameter String weatherDataFile =
    "D:/Desktop/MSC-PE/Thesis/THESIS ENS/modelica-ibpsa-master/IBPSA/Resources/weatherdata/DEU_Munich.108660_IWEC.mos"
    "Path to TMY3 weather data file"
    annotation(Dialog(
      group="Weather Data",
      tab="General"),
      choices(
        choice="D:/Desktop/MSC-PE/Thesis/THESIS ENS/modelica-ibpsa-master/IBPSA/Resources/weatherdata/DEU_Munich.108660_IWEC.mos"
          "Munich, Germany",
        choice="D:/Desktop/MSC-PE/Thesis/THESIS ENS/modelica-ibpsa-master/IBPSA/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"
          "San Francisco, USA",
        choice="D:/Desktop/MSC-PE/Thesis/THESIS ENS/modelica-ibpsa-master/IBPSA/Resources/weatherdata/USA_CO_Denver.Intl.AP.725650_TMY3.mos"
          "Denver, USA",
        choice="D:/Desktop/MSC-PE/Thesis/THESIS ENS/modelica-ibpsa-master/IBPSA/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"
          "Chicago, USA"));

  parameter Boolean useWeatherFile = false
    "If true, use weather file; if false, use T_ambient_fixed"
    annotation(Dialog(group="Weather Data", tab="General"));

  parameter Modelica.Units.SI.Temperature T_ambient_fixed=271.15
    "Fixed ambient temperature [K] (used when useWeatherFile=false)"
    annotation(Dialog(group="Weather Data", tab="General", enable=not useWeatherFile));

  // ═══════════════════════════════════════════════════════════════════
  // 🌡️ ZONE INITIAL TEMPERATURES - FOR COLD START TESTING
  // ═══════════════════════════════════════════════════════════════════
  parameter Modelica.Units.SI.Temperature TZoneInit_living = buildingData.TZoneInit_living
    "Initial living zone temperature [K] - Default from buildingData, set to 283.15 for cold start (10°C)"
    annotation(Dialog(group="Initial Conditions", tab="Advanced"));

  parameter Modelica.Units.SI.Temperature TZoneInit_cellar = buildingData.TZoneInit_cellar
    "Initial cellar zone temperature [K] - Default from buildingData, set to 283.15 for cold start (10°C)"
    annotation(Dialog(group="Initial Conditions", tab="Advanced"));

  parameter Modelica.Units.SI.Temperature TZoneInit_roof = buildingData.TZoneInit_roof
    "Initial roof zone temperature [K] - Default from buildingData, set to 283.15 for cold start (10°C)"
    annotation(Dialog(group="Initial Conditions", tab="Advanced"));

  // IBPSA WEATHER COMPONENTS
  IBPSA.BoundaryConditions.WeatherData.Bus weaBus if useWeatherFile
    "Weather data bus"
    annotation(Placement(
      visible=true,
      transformation(extent={{20,96},{40,116}},      rotation=0),
      iconTransformation(extent={{-110,80},{-90,100}}, rotation=0)));

  IBPSA.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(
    filNam=weatherDataFile,
    computeWetBulbTemperature=false) if useWeatherFile
    "Weather data reader"
    annotation(Placement(
      visible=true,
      transformation(extent={{-42,96},{-22,116}}, rotation=0),
      iconTransformation(extent={{-120,80},{-100,100}}, rotation=0)));

  // ═══════════════════════════════════════════════════════════════════
  // ☀️ IBPSA SOLAR RADIATION COMPONENTS (reference: SimpleRoomFourElements)
  // ═══════════════════════════════════════════════════════════════════
  IBPSA.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil[2](
    each til=1.5707963267949,
    each lat=0.87266462599716,
    azi={3.1415926535898,4.7123889803847}) if useWeatherFile
    "Direct solar radiation on tilted surfaces for 2 orientations (South, West)"
    annotation(Placement(
      visible=true,
      transformation(extent={{-4,146},{16,166}}, rotation=0)));

  IBPSA.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil[2](
    each outSkyCon=true,
    each outGroCon=true,
    each til=1.5707963267949,
    each lat=0.87266462599716,
    azi={3.1415926535898,4.7123889803847}) if useWeatherFile
    "Diffuse solar radiation on tilted surfaces for 2 orientations (South, West)"
    annotation(Placement(
      visible=true,
      transformation(extent={{-4,120},{16,140}}, rotation=0)));

  IBPSA.ThermalZones.ReducedOrder.SolarGain.CorrectionGDoublePane corGDouPan(
    UWin=2.1,
    n=2) if useWeatherFile
    "Correction factor for solar transmission through double-pane windows"
    annotation(Placement(
      visible=true,
      transformation(extent={{54,140},{74,160}}, rotation=0)));

  IBPSA.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow eqAirTemp(
    wfGro=0,
    withLongwave=true,
    aExt=0.7,
    alphaWallOut=20,
    alphaRad=5,
    alphaWinOut=20,
    n=2,
    wfWall={0.3043478260869566,0.6956521739130435},
    wfWin={0.5,0.5},
    TGro=285.15) if useWeatherFile
    "Equivalent air temperature for walls and windows"
    annotation(Placement(
      visible=true,
      transformation(extent={{102,120},{122,140}}, rotation=0)));

  Modelica.Blocks.Math.Add solRad[2] if useWeatherFile
    "Sum of direct and diffuse solar radiation per orientation"
    annotation(Placement(
      visible=true,
      transformation(extent={{28,126},{38,136}}, rotation=0)));

  Modelica.Blocks.Sources.Constant constSunblind[2](each k=0) if useWeatherFile
    "Sunblind signal: 0 = fully open (no shading)"
    annotation(Placement(
      visible=true,
      transformation(extent={{82,148},{92,158}}, rotation=0)));

  // ── Per-zone solar radiation mapping ──
  // Living zone: average of 2 orientations (corGDouPan output)
  Modelica.Blocks.Math.Add solRadLivingSum if useWeatherFile
    "Sum of solar radiation from both orientations for living zone"
    annotation(Placement(
      visible=true,
      transformation(extent={{102,162},{122,182}}, rotation=0)));

  Modelica.Blocks.Math.Gain solRadLivingAvg(k=0.5) if useWeatherFile
    "Average solar radiation for living zone (2 orientations → 1)"
    annotation(Placement(
      visible=true,
      transformation(extent={{134,162},{154,182}}, rotation=0)));

  // Living zone fallback: zero solar when weather file is not used
  Modelica.Blocks.Sources.Constant solRadLivingZero[1](each k=0) if not useWeatherFile
    "Zero solar radiation for living zone (no weather file)"
    annotation(Placement(
      visible=true,
      transformation(extent={{134,162},{154,182}}, rotation=0)));

  // Roof zone: zero solar (no vertical windows in roof model)
  Modelica.Blocks.Sources.Constant solRadRoofZero[1](each k=0)
    "Zero solar radiation for roof zone (roof has no vertical windows)"
    annotation(Placement(
      visible=true,
      transformation(extent={{134,190},{154,210}}, rotation=0)));

  // Cellar zone: zero solar (underground, no windows)
  Modelica.Blocks.Sources.Constant solRadCellarZero[1](each k=0)
    "Zero solar radiation for cellar zone (underground)"
    annotation(Placement(
      visible=true,
      transformation(extent={{176,120},{196,140}}, rotation=0)));

  inner Modelica.Fluid.System system(
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    T_ambient=T_ambient_fixed)
    "System-wide properties"
    annotation(Placement(
      visible=true,
      transformation(origin={-218,160}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-90,90}, extent={{-5,-5},{5,5}}, rotation=0)));

  // ═══════════════════════════════════════════════════════════════════
  // 📥 PHiL INPUTS
  // ═══════════════════════════════════════════════════════════════════
  Modelica.Blocks.Interfaces.RealInput STM_HCVLaM_degC(start=60)
    "Supply temperature from hardware [°C]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={-280,112}, extent={{-20,-20},{20,20}}, rotation=0),
        iconTransformation(origin={-110,90}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput SFW_HCRLbM_l_per_min(start=6)
    "Supply flow rate from hardware [L/min] (dummy for PHiL)"
    annotation(
      Placement(
        visible=true,
        transformation(origin={-280,70}, extent={{-20,-20},{20,20}}, rotation=0),
        iconTransformation(origin={-110,70}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput T_ambient_degC(start=5)
    "Ambient temperature [°C] (reference only)"
    annotation(
      Placement(
        visible=true,
        transformation(origin={-280,30}, extent={{-20,-20},{20,20}}, rotation=0),
        iconTransformation(origin={-110,50}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput nPersons_living_in(start=buildingData.nPersons_living_default)
    "Number of people in living room"
    annotation(
      Placement(
        visible=true,
        transformation(origin={-280,-20}, extent={{-20,-20},{20,20}}, rotation=0),
        iconTransformation(origin={-110,20}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput nPersons_cellar_in(start=buildingData.nPersons_cellar_default)
    "Number of people in cellar"
    annotation(
      Placement(
        visible=true,
        transformation(origin={-280,-60}, extent={{-20,-20},{20,20}}, rotation=0),
        iconTransformation(origin={-110,0}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput nPersons_roof_in(start=buildingData.nPersons_roof_default)
    "Number of people in roof office"
    annotation(
      Placement(
        visible=true,
        transformation(origin={-280,-100}, extent={{-20,-20},{20,20}}, rotation=0),
        iconTransformation(origin={-110,-20}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealInput P_appliances_living_W_in(start=buildingData.P_appliances_living_default)
    "Appliance power in living room [W]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,-130}, extent={{-20,-20},{20,20}}, rotation=180),
        iconTransformation(origin={110,-70}, extent={{-10,-10},{10,10}}, rotation=180)));

  Modelica.Blocks.Interfaces.RealInput P_appliances_cellar_W_in(start=buildingData.P_appliances_cellar_default)
    "Appliance power in cellar [W]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,-160}, extent={{-20,-20},{20,20}}, rotation=180),
        iconTransformation(origin={110,-80}, extent={{-10,-10},{10,10}}, rotation=180)));

  Modelica.Blocks.Interfaces.RealInput P_appliances_roof_W_in(start=buildingData.P_appliances_roof_default)
    "Appliance power in roof office [W]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,-190}, extent={{-20,-20},{20,20}}, rotation=180),
        iconTransformation(origin={110,-90}, extent={{-10,-10},{10,10}}, rotation=180)));

  // ═══════════════════════════════════════════════════════════════════
  // 📤 PHiL OUTPUTS
  // ═══════════════════════════════════════════════════════════════════
  Modelica.Blocks.Interfaces.RealOutput T_roomIs_degC
    "Living room temperature [°C]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,114}, extent={{-10,-10},{10,10}}, rotation=0),
        iconTransformation(origin={110,90}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput T_cellarIs_degC
    "Cellar temperature [°C]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,82}, extent={{-10,-10},{10,10}}, rotation=0),
        iconTransformation(origin={110,70}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput T_roofIs_degC
    "Roof office temperature [°C]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,50}, extent={{-10,-10},{10,10}}, rotation=0),
        iconTransformation(origin={110,50}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput STM_HCRL_Set_degC
    "Return temperature setpoint [°C]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,20}, extent={{-10,-10},{10,10}}, rotation=0),
        iconTransformation(origin={110,30}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput SFW_HCRLbM_Set_l_per_min
    "Calculated flow rate setpoint [L/min]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,-10}, extent={{-10,-10},{10,10}}, rotation=0),
        iconTransformation(origin={110,10}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_cellar_opening
    "Cellar valve position [0-1]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,-40}, extent={{-10,-10},{10,10}}, rotation=0),
        iconTransformation(origin={110,-10}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_living_opening
    "Living valve position [0-1]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,-70}, extent={{-10,-10},{10,10}}, rotation=0),
        iconTransformation(origin={110,-30}, extent={{-10,-10},{10,10}}, rotation=0)));

  Modelica.Blocks.Interfaces.RealOutput valve_roof_opening
    "Roof valve position [0-1]"
    annotation(
      Placement(
        visible=true,
        transformation(origin={270,-100}, extent={{-10,-10},{10,10}}, rotation=0),
        iconTransformation(origin={110,-50}, extent={{-10,-10},{10,10}}, rotation=0)));

  // ═══════════════════════════════════════════════════════════════════
  // 🔧 FLUID BOUNDARIES
  // ═══════════════════════════════════════════════════════════════════
  Modelica.Fluid.Sources.Boundary_pT supply(
    redeclare package Medium = Medium,
    use_p_in=false,
    p=250000,
    use_T_in=true,
    T=333.15,
    nPorts=1)
    "Supply boundary"
    annotation(Placement(
      visible=true,
      transformation(origin={-160,64}, extent={{-10,-10},{10,10}}, rotation=0),
      iconTransformation(origin={-80,60}, extent={{-5,-5},{5,5}}, rotation=0)));

  Modelica.Fluid.Sources.Boundary_pT return_sink(
    redeclare package Medium = Medium,
    p=150000,
    T=313.15,
    nPorts=1)
    "Return boundary"
    annotation(Placement(
      visible=true,
      transformation(origin={-160,-50}, extent={{-10,-10},{10,10}}, rotation=180),
      iconTransformation(origin={-80,-60}, extent={{-5,-5},{5,5}}, rotation=180)));

  Modelica.Fluid.Sensors.TemperatureTwoPort TReturn(
    redeclare package Medium = Medium)
    "Return temperature sensor"
    annotation(Placement(
      visible=true,
      transformation(origin={-100,-50}, extent={{10,-10},{-10,10}}, rotation=0),
      iconTransformation(origin={-60,-60}, extent={{-5,-5},{5,5}}, rotation=180)));

  // ═══════════════════════════════════════════════════════════════════
  // 🏢 BUILDING MODEL
  // ═══════════════════════════════════════════════════════════════════
  CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis.ThreeZoneBuilding_optimized building(
    redeclare package Medium = Medium,
    AZone_cellar=buildingData.AZone_cellar,
    AZone_living=buildingData.AZone_living,
    AZone_roof=buildingData.AZone_roof,
    hZone_cellar=buildingData.hZone_cellar,
    hZone_living=buildingData.hZone_living,
    hZone_roof=buildingData.hZone_roof,
    TZoneInit_cellar=TZoneInit_cellar,
    TZoneInit_living=TZoneInit_living,
    TZoneInit_roof=TZoneInit_roof,
    TRef_cellar=buildingData.TRef_cellar,
    TRef_living=buildingData.TRef_living,
    TRef_roof=buildingData.TRef_roof,
    k_PI=buildingData.k_PI,
    Ti_PI=buildingData.Ti_PI,
    yMin_cellar=buildingData.yMin_cellar,
    yMin_living=buildingData.yMin_living,
    yMin_roof=buildingData.yMin_roof,
    cellarHeat=buildingData.cellarHeat,
    roofHeat=buildingData.roofHeat)
    "Three-zone building"
    annotation(Placement(
      visible=true,
      transformation(extent={{-56,-110},{64,90}},rotation=0),
      iconTransformation(extent={{-40,-40},{40,40}}, rotation=0)));

  // ═══════════════════════════════════════════════════════════════════
  // 🔄 UNIT CONVERSIONS
  // ═══════════════════════════════════════════════════════════════════
  Modelica.Blocks.Sources.RealExpression toKelvin(y=STM_HCVLaM_degC + 273.15)
    "Convert supply temp from °C to K"
    annotation(Placement(
      visible=true,
      transformation(extent={{-220,54},{-180,74}}, rotation=0)));

  Modelica.Blocks.Sources.RealExpression kelvinOffset(y=273.15)
    "Kelvin offset for temperature conversions"
    annotation(Placement(
      visible=true,
      transformation(extent={{180,0},{200,20}}, rotation=0)));

  Modelica.Blocks.Math.Add toCelsius_living(k2=-1)
    "Convert living temp from K to °C"
    annotation(Placement(
      visible=true,
      transformation(extent={{220,104},{240,124}}, rotation=0)));

  Modelica.Blocks.Math.Add toCelsius_cellar(k2=-1)
    "Convert cellar temp from K to °C"
    annotation(Placement(
      visible=true,
      transformation(extent={{220,72},{240,92}}, rotation=0)));

  Modelica.Blocks.Math.Add toCelsius_roof(k2=-1)
    "Convert roof temp from K to °C"
    annotation(Placement(
      visible=true,
      transformation(extent={{220,40},{240,60}}, rotation=0)));

  Modelica.Blocks.Math.Add toCelsius_return(k2=-1)
    "Convert return temp from K to °C"
    annotation(Placement(
      visible=true,
      transformation(extent={{220,10},{240,30}}, rotation=0)));

  Modelica.Blocks.Math.Gain flowConvertOut(k=60000)
    "Convert flow from m³/s to L/min (1 m³/s = 60000 L/min)"
    annotation(Placement(
      visible=true,
      transformation(extent={{220,-20},{240,0}}, rotation=0)));

equation
  // Weather connection
  if useWeatherFile then
    connect(weaDat.weaBus, weaBus) annotation(
      Line(
        points={{-22,106},{30,106}},
        color={255,204,51},
        thickness=0.5));
  end if;

  // ═══════════════════════════════════════════════════════════════════
  // ☀️ SOLAR RADIATION CHAIN CONNECTIONS (reference: SimpleRoomFourElements)
  // ═══════════════════════════════════════════════════════════════════
  if useWeatherFile then
    // Weather bus → Solar irradiation components
    connect(weaDat.weaBus, HDirTil[1].weaBus)
      annotation(Line(
        points={{-22,106},{-14,106},{-14,156},{-4,156}},
        color={255,204,51},
        thickness=0.5));
    connect(weaDat.weaBus, HDirTil[2].weaBus)
      annotation(Line(
        points={{-22,106},{-14,106},{-14,156},{-4,156}},
        color={255,204,51},
        thickness=0.5));
    connect(weaDat.weaBus, HDifTil[1].weaBus)
      annotation(Line(
        points={{-22,106},{-14,106},{-14,130},{-4,130}},
        color={255,204,51},
        thickness=0.5));
    connect(weaDat.weaBus, HDifTil[2].weaBus)
      annotation(Line(
        points={{-22,106},{-14,106},{-14,130},{-4,130}},
        color={255,204,51},
        thickness=0.5));

    // Direct + Diffuse → solRad (sum per orientation)
    connect(HDirTil.H, solRad.u1)
      annotation(Line(
        points={{17,156},{22,156},{22,134},{27,134}},
        color={0,0,127}));
    connect(HDifTil.H, solRad.u2)
      annotation(Line(
        points={{17,130},{22,130},{22,128},{27,128}},
        color={0,0,127}));

    // solRad → eqAirTemp.HSol
    connect(solRad.y, eqAirTemp.HSol)
      annotation(Line(
        points={{38.5,131},{100,131},{100,136}},
        color={0,0,127}));

    // Weather bus → eqAirTemp
    connect(weaBus.TDryBul, eqAirTemp.TDryBul)
      annotation(Line(
        points={{30,106},{50,106},{50,124},{100,124}},
        color={255,204,51},
        thickness=0.5));
    connect(weaBus.TBlaSky, eqAirTemp.TBlaSky)
      annotation(Line(
        points={{30,106},{46,106},{46,130},{100,130}},
        color={255,204,51},
        thickness=0.5));

    // Sunblind → eqAirTemp
    connect(constSunblind.y, eqAirTemp.sunblind)
      annotation(Line(
        points={{92.5,153},{112,153},{112,142}},
        color={0,0,127}));

    // HDirTil, HDifTil → corGDouPan (window solar correction)
    connect(HDirTil.H, corGDouPan.HDirTil)
      annotation(Line(
        points={{17,156},{36,156},{52,156}},
        color={0,0,127}));
    connect(HDifTil.HSkyDifTil, corGDouPan.HSkyDifTil)
      annotation(Line(
        points={{17,136},{36,136},{36,151.8},{52,151.8},{52,152}},
        color={0,0,127}));
    connect(HDifTil.HGroDifTil, corGDouPan.HGroDifTil)
      annotation(Line(
        points={{17,124},{40,124},{40,148},{52,148}},
        color={0,0,127}));
    connect(HDirTil.inc, corGDouPan.inc)
      annotation(Line(
        points={{17,152},{44,152},{44,144},{52,144}},
        color={0,0,127}));

    // corGDouPan → per-zone solar radiation mapping
    // Living zone: average of 2 orientations
    connect(corGDouPan.solarRadWinTrans[1], solRadLivingSum.u1)
      annotation(Line(
        points={{75,150},{82,150},{82,178},{100,178}},
        color={0,0,127}));
    connect(corGDouPan.solarRadWinTrans[2], solRadLivingSum.u2)
      annotation(Line(
        points={{75,150},{86,150},{86,166},{100,166}},
        color={0,0,127}));
    connect(solRadLivingSum.y, solRadLivingAvg.u)
      annotation(Line(
        points={{123,172},{132,172}},
        color={0,0,127}));

    // Solar radiation → building zone inputs
    connect(solRadLivingAvg.y, building.solRad_living[1])
      annotation(Line(
        points={{155,172},{168,172},{168,86},{-68,86},{-68,-60},{-56,-60}},
        color={0,0,127}));
    connect(solRadRoofZero.y, building.solRad_roof)
      annotation(Line(
        points={{155,200},{172,200},{172,82},{-72,82},{-72,-30},{-56,-30}},
        color={0,0,127}));
    connect(solRadCellarZero.y, building.solRad_cellar)
      annotation(Line(
        points={{197,130},{202,130},{202,78},{-76,78},{-76,-90},{-56,-90}},
        color={0,0,127}));
  end if;

  // Fallback: when no weather file, connect zeros to all building solar inputs
  if not useWeatherFile then
    connect(solRadLivingZero.y, building.solRad_living)
      annotation(Line(
        points={{155,172},{168,172},{168,86},{-68,86},{-68,-60},{-56,-60}},
        color={0,0,127}));
    connect(solRadRoofZero.y, building.solRad_roof)
      annotation(Line(
        points={{155,200},{172,200},{172,82},{-72,82},{-72,-30},{-56,-30}},
        color={0,0,127}));
    connect(solRadCellarZero.y, building.solRad_cellar)
      annotation(Line(
        points={{197,130},{202,130},{202,78},{-76,78},{-76,-90},{-56,-90}},
        color={0,0,127}));
  end if;

  // Fluid connections
  connect(supply.ports[1], building.port_a) annotation (Line(points={{-150,64},{
    -76,64},{-76,50},{-56,50}}, color={0,127,255}, thickness=0.5, smooth=Smooth.Bezier));

  connect(building.port_b, TReturn.port_a) annotation(
    Line(
      points={{-56,-70},{-84,-70},{-84,-50},{-90,-50}},
      color={0,127,255},
      thickness=0.5,
      smooth=Smooth.Bezier));

  connect(TReturn.port_b, return_sink.ports[1]) annotation(
    Line(
      points={{-110,-50},{-170,-50}},
      color={0,127,255},
      thickness=0.5,
      smooth=Smooth.Bezier));

  connect(toKelvin.y, supply.T_in) annotation(
    Line(
      points={{-178,64},{-172,64},{-172,68}},
      color={0,0,127},
      smooth=Smooth.Bezier));

  // Temperature outputs
  connect(building.TZone_living, toCelsius_living.u1) annotation(
    Line(
      points={{70,20},{84,20},{84,120},{218,120}},
      color={0,0,127}));

  connect(kelvinOffset.y, toCelsius_living.u2) annotation(
    Line(
      points={{201,10},{210,10},{210,108},{218,108}},
      color={0,0,127}));

  connect(toCelsius_living.y, T_roomIs_degC) annotation(
    Line(
      points={{241,114},{270,114}},
      color={0,0,127}));

  connect(building.TZone_cellar, toCelsius_cellar.u1) annotation(
    Line(
      points={{70,50},{208,50},{208,88},{218,88}},
      color={0,0,127}));

  connect(kelvinOffset.y, toCelsius_cellar.u2) annotation(
    Line(
      points={{201,10},{210,10},{210,76},{218,76}},
      color={0,0,127}));

  connect(toCelsius_cellar.y, T_cellarIs_degC) annotation(
    Line(
      points={{241,82},{270,82}},
      color={0,0,127}));

  connect(building.TZone_roof, toCelsius_roof.u1) annotation(
    Line(
      points={{70,-10},{174,-10},{174,56},{218,56}},
      color={0,0,127}));

  connect(kelvinOffset.y, toCelsius_roof.u2) annotation(
    Line(
      points={{201,10},{210,10},{210,44},{218,44}},
      color={0,0,127}));

  connect(toCelsius_roof.y, T_roofIs_degC) annotation(
    Line(
      points={{241,50},{270,50}},
      color={0,0,127}));

  connect(TReturn.T, toCelsius_return.u1) annotation(
    Line(
      points={{-100,-39},{-100,-32},{206,-32},{206,26},{218,26}},
      color={0,0,127}));

  connect(kelvinOffset.y, toCelsius_return.u2) annotation(
    Line(
      points={{201,10},{210,10},{210,14},{218,14}},
      color={0,0,127}));

  connect(toCelsius_return.y, STM_HCRL_Set_degC) annotation(
    Line(
      points={{241,20},{270,20}},
      color={0,0,127}));

  // Flow rate output - FIXED CONVERSION!
  connect(building.qvRef, flowConvertOut.u) annotation(
    Line(
      points={{70,-80},{208,-80},{208,-10},{218,-10}},
      color={0,0,127}));

  connect(flowConvertOut.y, SFW_HCRLbM_Set_l_per_min) annotation(
    Line(
      points={{241,-10},{270,-10}},
      color={0,0,127}));

  // Valve outputs
  connect(building.valve_cellar_opening, valve_cellar_opening) annotation(
    Line(
      points={{70,-40},{270,-40}},
      color={0,0,127}));

  connect(building.valve_living_opening, valve_living_opening) annotation(
    Line(
      points={{70,-60},{254,-60},{254,-70},{270,-70}},
      color={0,0,127}));

  connect(building.valve_roof_opening, valve_roof_opening) annotation(
    Line(
      points={{70,-100},{270,-100}},
      color={0,0,127}));

  // Occupancy inputs
  connect(nPersons_living_in, building.nPersons_living) annotation(
    Line(
      points={{-280,-20},{-78,-20},{-78,40},{-62,40}},
      color={0,0,127}));

  connect(nPersons_cellar_in, building.nPersons_cellar) annotation(
    Line(
      points={{-280,-60},{-252,-60},{-252,82},{-76,82},{-76,70},{-62,70}},
      color={0,0,127}));

  connect(nPersons_roof_in, building.nPersons_roof) annotation(
    Line(
      points={{-280,-100},{-280,-102},{-188,-102},{-188,10},{-62,10}},
      color={0,0,127}));

  // Appliance inputs
  connect(P_appliances_living_W_in, building.P_appliances_living_W) annotation(
    Line(
      points={{270,-130},{244,-130},{244,-38},{176,-38},{176,-6},{88,-6},{88,40},
          {70,40}},
      color={0,0,127}));

  connect(P_appliances_cellar_W_in, building.P_appliances_cellar_W) annotation(
    Line(
      points={{270,-160},{270,-40},{254,-40},{254,-34},{206,-34},{206,70},{70,70}},
      color={0,0,127}));

  connect(P_appliances_roof_W_in, building.P_appliances_roof_W) annotation(
    Line(
      points={{270,-190},{270,-160},{84,-160},{84,10},{70,10}},
      color={0,0,127}));


annotation(
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-80,70},{80,-70}},
          lineColor={0,0,255},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Polygon(
          points={{-90,70},{0,95},{90,70},{-90,70}},
          lineColor={139,69,19},
          fillColor={205,133,63},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,95},{75,85}},
          lineColor={200,200,200},
          fillColor={230,230,230},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{65,92},{82,80}},
          lineColor={200,200,200},
          fillColor={230,230,230},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,95},{85,82}},
          lineColor={200,200,200},
          fillColor={230,230,230},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,60},{-80,55}},
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,40},{-80,35}},
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,20},{-80,15}},
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{80,60},{100,55}},
          lineColor={255,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{80,40},{100,35}},
          lineColor={255,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{80,20},{100,15}},
          lineColor={255,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-75,60},{75,35}},
          textString="PHiL Wrapper",
          textColor={0,0,0},
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-75,25},{75,5}},
          textString="%name",
          textColor={0,0,127}),
        Text(
          extent={{-75,-5},{75,-25}},
          textString=DynamicSelect("SmallHouse", String(buildingData)),
          textColor={28,108,200},
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-75,-35},{75,-55}},
          textString="Weather Ready",
          textColor={100,100,100},
          fontSize=8),
        Text(
          extent={{-75,-60},{75,-75}},
          textString="60000× Flow",
          textColor={0,127,0},
          fontSize=8)}
        // White background

        // Main building rectangle

        // Building roof (triangle)

        // Weather indicator - small cloud in corner

        // PHiL connectors (left side - inputs)

        // PHiL connectors (right side - outputs)

        // Text labels - well spaced, no overlap

        // Dynamic building type display

        // Weather indicator text

        // Flow conversion indicator
),

    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-280,-200},{280,220}})),

    Documentation(info="<html>
<h4>PHiL Wrapper with Weather Integration</h4>
<p><b>✅ CRITICAL FIX: Flow Conversion Factor</b></p>
<p>flowConvertOut gain = 60000 (converts m³/s to L/min)</p>
<p><b>✅ Weather File Dropdown</b></p>
<p>Select from 4 pre-configured weather files:</p>
<ul>
<li>Munich, Germany</li>
<li>San Francisco, USA</li>
<li>Denver, USA</li>
<li>Chicago, USA</li>
</ul>
<p><b>✅ Full Annotations</b></p>
<p>All connections have proper routing and colors</p>
<p><b>✅ Building Type Dropdown</b></p>
<p>Select: SmallHouse, BigHouse, Office, Hospital, School</p>
<p><b>✅ TZoneInit Parameters Exposed</b></p>
<p>Override initial temperatures for cold start testing</p>
<p><b>FOR FMU EXPORT: Use this model!</b></p>
</html>"));

end ThreeZoneBuilding_PHiL_optimized_withWeather;
