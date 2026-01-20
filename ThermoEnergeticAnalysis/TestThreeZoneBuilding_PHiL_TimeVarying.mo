within CoSES_Thermal_ProHMo_PHiL.ThermoEnergeticAnalysis;
model TestThreeZoneBuilding_PHiL_TimeVarying
  "Test model with TIME-VARYING occupancy and appliance schedules"

  // ============================================================================
  // THE PHiL MODEL
  // ============================================================================

  ThreeZoneBuilding_PHiL_basic building(
    AZone_cellar=80,
    AZone_living=100,
    AZone_roof=60,
    hZone_cellar=2.2,
    hZone_living=2.5,
    hZone_roof=2.3,
    TZoneInit_cellar=288.15,
    TZoneInit_living=294.15,
    TZoneInit_roof=289.15,
    TRef_cellar=288.15,
    TRef_living=294.15,
    TRef_roof=294.15,
    TOutdoor=278.15,
    cellarHeat=true,
    roofHeat=true)
    annotation (Placement(transformation(extent={{-40,-40},{40,40}})));

  // ============================================================================
  // PHiL INPUT SOURCES (from CoSES Lab) - TOP LEFT
  // ============================================================================

  Modelica.Blocks.Sources.Constant supplyTemp(k=60)
    "Supply water temperature [deg C]"
    annotation (Placement(transformation(extent={{-200,140},{-180,160}})));

  Modelica.Blocks.Sources.Constant flowRate(k=6)
    "Volume flow rate [L/min]"
    annotation (Placement(transformation(extent={{-200,110},{-180,130}})));

  Modelica.Blocks.Sources.Constant ambientTemp(k=5)
    "Outdoor temperature [deg C]"
    annotation (Placement(transformation(extent={{-200,80},{-180,100}})));

  // ============================================================================
  // OCCUPANCY SCHEDULES - MIDDLE LEFT
  // ============================================================================

  Modelica.Blocks.Sources.CombiTimeTable nPersons_living_schedule(
    table=[
      0,      2;
      21600,  2;
      28800,  0;
      43200,  0;
      64800,  0;
      64801,  2;
      79200,  2;
      86400,  2],     // 00:00 - 2 persons (sleeping)
                      // 06:00 - 2 persons (waking up)
                      // 08:00 - 0 persons (left for work)
                      // 12:00 - 0 persons (at work)
                      // 18:00 - 0 persons (coming home)
                      // 18:00 - 2 persons (arrived home)
                      // 22:00 - 2 persons (evening)
                      // 24:00 - 2 persons (end of day)
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments)
    "Living zone: 2 persons morning/evening, 0 during work hours"
    annotation (Placement(transformation(extent={{-200,20},{-180,40}})));

  Modelica.Blocks.Sources.CombiTimeTable nPersons_cellar_schedule(
    table=[
      0,      0;
      72000,  0;
      72001,  1;
      79200,  1;
      79201,  0;
      86400,  0],     // 00:00 - 0 persons
                      // 20:00 - 0 persons
                      // 20:00 - 1 person (hobby time in cellar)
                      // 22:00 - 1 person
                      // 22:00 - 0 persons (back upstairs)
                      // 24:00 - 0 persons
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments)
    "Cellar: 1 person 8pm-10pm (hobby workshop)"
    annotation (Placement(transformation(extent={{-200,-10},{-180,10}})));

  Modelica.Blocks.Sources.CombiTimeTable nPersons_roof_schedule(
    table=[
      0,      0;
      32400,  0;
      32401,  1;
      43200,  1;
      64800,  1;
      64801,  0;
      86400,  0],     // 00:00 - 0 persons
                      // 09:00 - 0 persons
                      // 09:00 - 1 person (home office starts)
                      // 12:00 - 1 person (lunch at desk)
                      // 18:00 - 1 person (finishing work)
                      // 18:00 - 0 persons (done working)
                      // 24:00 - 0 persons
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments)
    "Roof: 1 person 9am-6pm (home office)"
    annotation (Placement(transformation(extent={{-200,-40},{-180,-20}})));

  // ============================================================================
  // APPLIANCE SCHEDULES - BOTTOM LEFT
  // ============================================================================

  Modelica.Blocks.Sources.CombiTimeTable P_app_living_schedule(
    table=[
      0,      30;
      21600,  30;
      21601,  200;
      28800,  200;
      28801,  50;
      64800,  50;
      64801,  400;
      79200,  400;
      79201,  150;
      82800,  150;
      82801,  30;
      86400,  30],    // 00:00 - 30W standby (router, clocks)
                      // 06:00 - 30W standby
                      // 06:00 - 200W (lights, coffee maker, toaster)
                      // 08:00 - 200W morning
                      // 08:00 - 50W standby (everyone left)
                      // 18:00 - 50W standby
                      // 18:00 - 400W (TV, cooking, lights)
                      // 22:00 - 400W evening
                      // 22:00 - 150W (just TV)
                      // 23:00 - 150W
                      // 23:00 - 30W standby (bedtime)
                      // 24:00 - 30W
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments)
    "Living: high morning/evening, low during day"
    annotation (Placement(transformation(extent={{-202,-90},{-182,-70}})));

  Modelica.Blocks.Sources.CombiTimeTable P_app_cellar_schedule(
    table=[
      0,      80;
      72000,  80;
      72001,  280;
      79200,  280;
      79201,  80;
      86400,  80],    // 00:00 - 80W (freezer always on)
                      // 20:00 - 80W
                      // 20:00 - 280W (freezer + workshop tools)
                      // 22:00 - 280W
                      // 22:00 - 80W (back to freezer only)
                      // 24:00 - 80W
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments)
    "Cellar: 80W base (freezer) + 200W during hobby time"
    annotation (Placement(transformation(extent={{-200,-118},{-180,-98}})));

  Modelica.Blocks.Sources.CombiTimeTable P_app_roof_schedule(
    table=[
      0,      20;
      32400,  20;
      32401,  250;
      43200,  250;
      64800,  250;
      64801,  20;
      86400,  20],    // 00:00 - 20W standby
                      // 09:00 - 20W
                      // 09:00 - 250W (computer, monitors, desk lamp)
                      // 12:00 - 250W
                      // 18:00 - 250W
                      // 18:00 - 20W standby (work done)
                      // 24:00 - 20W
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments)
    "Roof: 250W during work hours (computer), 20W standby"
    annotation (Placement(transformation(extent={{-200,-148},{-180,-128}})));

  // ============================================================================
  // TIME CONVERSION (for X-axis in hours/days)
  // ============================================================================
  Real time_hours = time/3600 "Simulation time [hours]";
  Real time_days = time/86400 "Simulation time [days]";
  Real hour_of_day = mod(time/3600, 24) "Hour of day (0-24) [hours]";

  // ============================================================================
  // OUTPUT ALIASES - TEMPERATURES (for easy plotting)
  // ============================================================================

  Real T_living_degC = building.T_roomIs_degC "Living zone temperature [deg C]";
  Real T_cellar_degC = building.T_cellarIs_degC "Cellar temperature [deg C]";
  Real T_roof_degC = building.T_roofIs_degC "Roof temperature [deg C]";
  Real T_return_degC = building.STM_HCRL_Set_degC "Return water temperature [deg C]";
  Real qv_l_per_min = building.SFW_HCRLbM_Set_l_per_min "Flow rate [L/min]";

  // ============================================================================
  // OUTPUT ALIASES - INTERNAL GAINS (for plotting)
  // ============================================================================

  Real nPersons_living = nPersons_living_schedule.y[1] "Persons in living zone";
  Real nPersons_cellar = nPersons_cellar_schedule.y[1] "Persons in cellar";
  Real nPersons_roof = nPersons_roof_schedule.y[1] "Persons in roof";
  Real P_app_living_W = P_app_living_schedule.y[1] "Appliance power living [W]";
  Real P_app_cellar_W = P_app_cellar_schedule.y[1] "Appliance power cellar [W]";
  Real P_app_roof_W = P_app_roof_schedule.y[1] "Appliance power roof [W]";

  // Total internal gains per zone
  Real Q_gains_living_W = nPersons_living * 80 + P_app_living_W "Total gains living [W]";
  Real Q_gains_cellar_W = nPersons_cellar * 80 + P_app_cellar_W "Total gains cellar [W]";
  Real Q_gains_roof_W = nPersons_roof * 80 + P_app_roof_W "Total gains roof [W]";
  Real Q_gains_total_W = Q_gains_living_W + Q_gains_cellar_W + Q_gains_roof_W "Total building gains [W]";

equation
  // ============================================================================
  // PHiL INPUT CONNECTIONS
  // ============================================================================
  connect(supplyTemp.y, building.STM_HCVLaM_degC)
    annotation (Line(points={{-179,150},{-100,150},{-100,22.4},{-36.9231,22.4}},
                                                                        color={0,0,127}));
  connect(flowRate.y, building.SFW_HCRLbM_l_per_min)
    annotation (Line(points={{-179,120},{-110,120},{-110,9.6},{-36.9231,9.6}},
                                                                        color={0,0,127}));
  connect(ambientTemp.y, building.T_ambient_degC)
    annotation (Line(points={{-179,90},{-120,90},{-120,2.4},{-36.9231,2.4}},
                                                                      color={0,0,127}));

  // ============================================================================
  // OCCUPANCY CONNECTIONS
  // ============================================================================
  connect(nPersons_living_schedule.y[1], building.nPersons_living_in)
    annotation (Line(points={{-179,30},{-130,30},{-130,0},{-40,0}}, color={0,0,127}));
  connect(nPersons_cellar_schedule.y[1], building.nPersons_cellar_in)
    annotation (Line(points={{-179,0},{-140,0},{-140,-12},{-40,-12}},
                                                                    color={0,0,127}));
  connect(nPersons_roof_schedule.y[1], building.nPersons_roof_in)
    annotation (Line(points={{-179,-30},{-130,-30},{-130,12},{-40,12}},   color={0,0,127}));

  // ============================================================================
  // APPLIANCE CONNECTIONS
  // ============================================================================
  connect(P_app_living_schedule.y[1], building.P_appliances_living_W_in)
    annotation (Line(points={{-181,-80},{-120,-80},{-120,0},{40,0}},      color={255,128,0}));
  connect(P_app_cellar_schedule.y[1], building.P_appliances_cellar_W_in)
    annotation (Line(points={{-179,-108},{-110,-108},{-110,-12},{40,-12}},  color={255,128,0}));
  connect(P_app_roof_schedule.y[1], building.P_appliances_roof_W_in)
    annotation (Line(points={{-179,-138},{-100,-138},{-100,12},{40,12}},    color={255,128,0}));

  annotation (
    experiment(
      StartTime=0,
      StopTime=259200,
      Interval=60,
      Tolerance=1e-06),
    Diagram(
      coordinateSystem(preserveAspectRatio=false, extent={{-220,-160},{120,180}}),
      graphics={
        Rectangle(
          extent={{-218,178},{-170,70}},
          lineColor={0,0,255},
          fillColor={230,240,255},
          fillPattern=FillPattern.Solid,
          radius=5),
        Rectangle(
          extent={{-218,60},{-170,-50}},
          lineColor={0,128,0},
          fillColor={230,255,230},
          fillPattern=FillPattern.Solid,
          radius=5),
        Rectangle(
          extent={{-218,-54},{-170,-150}},
          lineColor={255,128,0},
          fillColor={255,245,230},
          fillPattern=FillPattern.Solid,
          radius=5),
        Text(
          extent={{-216,176},{-172,164}},
          textColor={0,0,255},
          textString="PHiL Inputs",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-216,58},{-172,46}},
          textColor={0,128,0},
          textString="Occupancy",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-214,-54},{-170,-66}},
          textColor={255,128,0},
          textString="Appliances",
          textStyle={TextStyle.Bold}),
        Text(extent={{-168,162},{-100,150}}, textColor={0,0,255}, textString="T_supply = 60°C"),
        Text(extent={{-168,132},{-100,120}}, textColor={0,0,255}, textString="Flow = 6 L/min"),
        Text(extent={{-168,102},{-100,90}},textColor={0,0,255}, textString="T_ambient = 5°C"),
        Text(extent={{-168,42},{-80,30}}, textColor={0,128,0}, textString="Living: 0-2 persons"),
        Text(extent={{-168,12},{-80,0}}, textColor={0,128,0}, textString="Cellar: 0-1 person"),
        Text(extent={{-168,-16},{-80,-28}}, textColor={0,128,0}, textString="Roof: 0-1 person"),
        Text(extent={{-168,-64},{-84,-76}}, textColor={255,128,0}, textString="Living: 30-400 W"),
        Text(extent={{-170,-94},{-82,-106}}, textColor={255,128,0}, textString="Cellar: 80-280 W"),
        Text(extent={{-174,-126},{-86,-136}}, textColor={255,128,0}, textString="Roof: 20-250 W"),
        Text(
          extent={{-160,182},{80,164}},
          textColor={0,0,0},
          textString="PHiL Building Test - Variable Internal Gains",
          textStyle={TextStyle.Bold})}
        // ====== BACKGROUND SECTIONS ======

        // ====== SECTION LABELS ======

        // ====== INPUT LABELS ======

        // ====== TITLE ======

        // ====== OUTPUT LABELS ======
),  Icon(
      coordinateSystem(preserveAspectRatio=false, extent={{-200,-200},{200,200}}),
      graphics={
        Rectangle(
          extent={{-200,200},{200,-200}},
          lineColor={135,206,235},
          fillColor={135,206,235},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-200,-120},{200,-200}},
          lineColor={34,139,34},
          fillColor={34,139,34},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{120,180},{180,120}},
          lineColor={255,215,0},
          fillColor={255,215,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-140,-120},{140,40}},
          lineColor={139,69,19},
          fillColor={255,228,196},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Polygon(
          points={{-160,40},{0,140},{160,40},{-160,40}},
          lineColor={139,69,19},
          fillColor={178,34,34},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Rectangle(
          extent={{-120,-120},{120,-160}},
          lineColor={105,105,105},
          fillColor={169,169,169},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-100,-125},{100,-155}},
          textColor={255,255,255},
          textString="CELLAR",
          textStyle={TextStyle.Bold}),
        Rectangle(
          extent={{-120,-100},{120,20}},
          lineColor={0,0,0},
          fillColor={255,250,205},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-100,-20},{100,-60}},
          textColor={0,0,0},
          textString="LIVING",
          textStyle={TextStyle.Bold}),
        Polygon(
          points={{-100,30},{0,100},{100,30},{-100,30}},
          lineColor={0,0,0},
          fillColor={255,218,185},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-60,80},{60,40}},
          textColor={0,0,0},
          textString="ROOF",
          textStyle={TextStyle.Bold}),
        Rectangle(
          extent={{-20,-100},{20,-40}},
          lineColor={139,69,19},
          fillColor={139,69,19},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{8,-65},{14,-71}},
          lineColor={255,215,0},
          fillColor={255,215,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-20},{-50,10}},
          lineColor={0,0,139},
          fillColor={173,216,230},
          fillPattern=FillPattern.Solid),
        Line(points={{-75,-20},{-75,10}}, color={0,0,139}),
        Line(points={{-100,-5},{-50,-5}}, color={0,0,139}),
        Rectangle(
          extent={{50,-20},{100,10}},
          lineColor={0,0,139},
          fillColor={173,216,230},
          fillPattern=FillPattern.Solid),
        Line(points={{75,-20},{75,10}}, color={0,0,139}),
        Line(points={{50,-5},{100,-5}}, color={0,0,139}),
        Ellipse(
          extent={{-20,50},{20,80}},
          lineColor={0,0,139},
          fillColor={173,216,230},
          fillPattern=FillPattern.Solid),
        Line(points={{0,50},{0,80}}, color={0,0,139}),
        Line(points={{-20,65},{20,65}}, color={0,0,139}),
        Rectangle(
          extent={{60,100},{90,160}},
          lineColor={139,69,19},
          fillColor={178,34,34},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-200,80},{-140,80},{-140,0}},
          color={255,0,0},
          thickness=1),
        Polygon(
          points={{-145,10},{-135,10},{-140,0},{-145,10}},
          lineColor={255,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-140,-80},{-140,-140},{-200,-140}},
          color={0,0,255},
          thickness=1),
        Polygon(
          points={{-190,-135},{-190,-145},{-200,-140},{-190,-135}},
          lineColor={0,0,255},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{140,60},{200,60}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{140,-40},{200,-40}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{120,-140},{200,-140}},
          color={238,46,47},
          thickness=0.5),
        Ellipse(
          extent={{-90,-70},{-80,-80}},
          lineColor={0,0,0},
          fillColor={255,200,150},
          fillPattern=FillPattern.Solid),
        Line(points={{-85,-80},{-85,-95}}, color={0,0,0}),
        Line(points={{-85,-85},{-92,-90}}, color={0,0,0}),
        Line(points={{-85,-85},{-78,-90}}, color={0,0,0}),
        Ellipse(
          extent={{-70,-70},{-60,-80}},
          lineColor={0,0,0},
          fillColor={255,200,150},
          fillPattern=FillPattern.Solid),
        Line(points={{-65,-80},{-65,-95}}, color={0,0,0}),
        Line(points={{-65,-85},{-72,-90}}, color={0,0,0}),
        Line(points={{-65,-85},{-58,-90}}, color={0,0,0}),
        Rectangle(
          extent={{60,-60},{90,-85}},
          lineColor={0,0,0},
          fillColor={50,50,50},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{65,-63},{85,-80}},
          lineColor={100,100,255},
          fillColor={100,100,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-198,100},{-142,86}},
          textColor={255,0,0},
          textString="T_supply",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-198,-120},{-142,-134}},
          textColor={0,0,255},
          textString="T_return",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{142,74},{198,60}},
          textColor={238,46,47},
          textString="T_roof"),
        Text(
          extent={{142,-26},{198,-40}},
          textColor={238,46,47},
          textString="T_living"),
        Text(
          extent={{142,-126},{198,-140}},
          textColor={238,46,47},
          textString="T_cellar"),
        Text(
          extent={{-180,198},{180,162}},
          textColor={0,0,0},
          textString="Three Zone Building - PHiL",
          textStyle={TextStyle.Bold}),
        Text(
          extent={{-180,-170},{180,-195}},
          textColor={0,0,128},
          textString="Variable Internal Gains")}
        // ====== SKY BACKGROUND ======

        // ====== GROUND ======

        // ====== SUN ======

        // ====== HOUSE MAIN BODY ======

        // ====== ROOF ======

        // ====== CELLAR (below ground) ======

        // ====== LIVING ZONE ======

        // ====== ROOF ZONE (attic) ======

        // ====== DOOR ======

        // ====== WINDOWS ======

        // ====== ATTIC WINDOW ======

        // ====== CHIMNEY ======

        // ====== HOT WATER PIPE (Input) ======

        // ====== COLD WATER PIPE (Output) ======

        // ====== TEMPERATURE OUTPUT ARROWS ======

        // ====== PERSON ICONS ======

        // ====== APPLIANCE ICON (TV) ======

        // ====== INPUT/OUTPUT LABELS ======

        // ====== TITLE ======
),  Documentation(info="<html>
<h4>Time-Varying Internal Gains Test Model</h4>

<h5>Daily Schedule Summary:</h5>
<table border=\"1\">
<tr><th>Time</th><th>Living</th><th>Cellar</th><th>Roof</th></tr>
<tr><td>00:00-06:00</td><td>2 pers + 30W</td><td>0 pers + 80W</td><td>0 pers + 20W</td></tr>
<tr><td>06:00-08:00</td><td>2 pers + 200W</td><td>0 pers + 80W</td><td>0 pers + 20W</td></tr>
<tr><td>08:00-09:00</td><td>0 pers + 50W</td><td>0 pers + 80W</td><td>0 pers + 20W</td></tr>
<tr><td>09:00-18:00</td><td>0 pers + 50W</td><td>0 pers + 80W</td><td>1 pers + 250W</td></tr>
<tr><td>18:00-20:00</td><td>2 pers + 400W</td><td>0 pers + 80W</td><td>0 pers + 20W</td></tr>
<tr><td>20:00-22:00</td><td>2 pers + 400W</td><td>1 pers + 280W</td><td>0 pers + 20W</td></tr>
<tr><td>22:00-23:00</td><td>2 pers + 150W</td><td>0 pers + 80W</td><td>0 pers + 20W</td></tr>
<tr><td>23:00-24:00</td><td>2 pers + 30W</td><td>0 pers + 80W</td><td>0 pers + 20W</td></tr>
</table>

<h5>Variables to Plot:</h5>
<ul>
<li>Temperatures: T_living_degC, T_cellar_degC, T_roof_degC</li>
<li>Occupancy: nPersons_living, nPersons_cellar, nPersons_roof</li>
<li>Appliances: P_app_living_W, P_app_cellar_W, P_app_roof_W</li>
<li>Total gains: Q_gains_living_W, Q_gains_cellar_W, Q_gains_roof_W, Q_gains_total_W</li>
</ul>
</html>"));
end TestThreeZoneBuilding_PHiL_TimeVarying;
