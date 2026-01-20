within CoSES_Thermal_ProHMo_PHiL.BuildingSystem.Building;
model TestHydronicZone
  inner Modelica.Fluid.System system;
  replaceable package Medium = Modelica.Media.Water.StandardWater
    annotation(Placement(transformation(extent={{-100,80},{-60,100}})));
  // Zone under test

  // Hydronic system
  HydronicSystem.SystemWithZoneAndHydronics hydSys(
    redeclare package Medium = Medium // Modelica.Media.Water.StandardWater  //  10 Dec 18.00 IBPSA.Media.Water)
)   annotation(Placement(transformation(extent={{-40,-20},{0,20}})));

  // Supply boundary (acts like boiler supply)
  Modelica.Fluid.Sources.Boundary_pT supply(
    redeclare package Medium = Medium, // IBPSA.Media.Water,
      p = 300000,
      T = 333.15,
      nPorts = 1)
    annotation(Placement(transformation(extent={{-100,-10},{-80,10}})));

  // 13 Dec 17.00 Return boundary (boiler return) - use MassFlowSource to avoid pressure conflict
  Modelica.Fluid.Sources.MassFlowSource_T ret(
    redeclare package Medium = Medium,
      use_m_flow_in = false,
      m_flow = -0.05,
      T = 303.15,
      nPorts = 1)
    annotation(Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
      origin={10,-82})));
  /* // 13 Dec 17.00 commenteds
  // Return boundary (boiler return)
  Modelica.Fluid.Sources.Boundary_pT ret(
    redeclare package Medium = Medium, // IBPSA.Media.Water,
      p = 280000,
      T = 303.15,
      nPorts = 1)s
    annotation(Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={10,-82})));
*/


  // 13 Dec 17.00 added TZoneRefConst & WindowShadingConst
  // Constant inputs for zone
  Modelica.Blocks.Sources.Constant TZoneRefConst(k=293.15)
    "Reference zone temperature 20°C"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={64,-12})));

  Modelica.Blocks.Sources.Constant WindowShadingConst[3](each k=0)
    "No window shading"
    annotation (Placement(transformation(extent={{16,-22},{36,-2}})));
  Modelica.Blocks.Sources.Constant TBoundOutConst[3](each k=283.15)
    annotation (Placement(transformation(extent={{-22,40},{-2,60}})));
equation
  /* // 15 Dec 11.00 commented
  // 15 Dec 00.00 Tie interior surface temps to zone air
for i in 1:3 loop
  HeatedZone.Boundaries[i].TBoundIn = HeatedZone.TZone;
end for;
*/

  // Water loop
  connect(supply.ports[1], hydSys.port_a)
    annotation (Line(points={{-80,0},{-37.6,0}},
                                               color={0,127,255}));

  connect(hydSys.port_b, ret.ports[1])
    annotation (Line(points={{-2,0},{6,0},{6,-72},{10,-72}},
                                                         color={0,127,255}));

  // Thermal connection to zone
  // 16 Dec 13.00 connect(hydSys.port_zone, HeatedZone.portInt)
  connect(hydSys.port_zone, HeatedZone.heatPort)
    annotation (Line(points={{-20,18},{-20,26.2},{59.4,26.2}},
                                                         color={191,0,0}));
  connect(WindowShadingConst[1:3].y, HeatedZone.WindowShading[1:3]) annotation
    (Line(points={{37,-12},{50,-12},{50,12.1},{59.7,12.1}}, color={0,0,127}));
  connect(TZoneRefConst.y, HeatedZone.TZoneRef) annotation (Line(points={{75,
          -12},{86,-12},{86,26},{79.8,26}}, color={0,0,127}));
end TestHydronicZone;
