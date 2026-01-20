within CoSES_Thermal_ProHMo_PHiL.BuildingSystem.HeatingSystem;
package Control
  model volumeFlowControlHeatingSystem
    Modelica.Blocks.Continuous.PID pidTemp(
      k=1,
      Ti=200,
      Td=0)
      annotation (Placement(transformation(extent={{-48,-14},{-28,6}})));
    Modelica.Blocks.Math.Division divFlow
      annotation (Placement(transformation(extent={{4,-20},{24,0}})));
    Modelica.Blocks.Math.Gain gainRhoCp(k=4.17e6)
      annotation (Placement(transformation(extent={{-24,-54},{-4,-34}})));
    Modelica.Blocks.Math.Gain gainNormalize(k=50)
      annotation (Placement(transformation(extent={{40,-18},{60,2}})));
    Modelica.Blocks.Math.Add tempError(k2=-1)
      annotation (Placement(transformation(extent={{-92,-14},{-72,6}})));
    Modelica.Blocks.Math.Add deltaT(k2=-1)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=270, origin={-8,54})));
    Modelica.Blocks.Nonlinear.Limiter limitFlow(uMax=1, uMin=0)
      annotation (Placement(transformation(extent={{78,-12},{98,8}})));
    Modelica.Blocks.Interfaces.RealInput TRef "Reference temp"
      annotation (Placement(transformation(extent={{-176,-58},{-136,-18}})));
    Modelica.Blocks.Interfaces.RealInput TAct "Actual temp" annotation (
        Placement(transformation(extent={{-172,22},{-132,62}})));
    Modelica.Blocks.Interfaces.RealInput TReturn "Return temp"
      annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=270,
          origin={-40,112})));
    Modelica.Blocks.Interfaces.RealInput TFlow "Flow temp" annotation (
        Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=270,
          origin={40,116})));
    Modelica.Blocks.Interfaces.RealOutput qvRef annotation (Placement(
          transformation(extent={{126,-10},{150,14}}), iconTransformation(
            extent={{126,-10},{150,14}})));
  equation
    connect(tempError.u1, TRef) annotation (Line(points={{-94,2},{-122,2},
            {-122,-38},{-156,-38}}, color={0,0,127}));
    connect(TAct, tempError.u2) annotation (Line(points={{-152,42},{-152,
            -10},{-94,-10}}, color={0,0,127}));
    connect(pidTemp.u, tempError.y)
      annotation (Line(points={{-50,-4},{-71,-4}}, color={0,0,127}));
    connect(pidTemp.y, divFlow.u1)
      annotation (Line(points={{-27,-4},{2,-4}}, color={0,0,127}));
    connect(TFlow, deltaT.u1) annotation (Line(points={{40,116},{40,66},{
            -2,66}}, color={0,0,127}));
    connect(TReturn, deltaT.u2) annotation (Line(points={{-40,112},{-40,
            66},{-14,66}}, color={0,0,127}));
    connect(divFlow.y, gainNormalize.u) annotation (Line(points={{25,-10},
            {30,-8},{38,-8}}, color={0,0,127}));
    connect(gainNormalize.y, limitFlow.u) annotation (Line(points={{61,-8},
            {66,-8},{66,-2},{76,-2}}, color={0,0,127}));
    connect(deltaT.y, gainRhoCp.u) annotation (Line(points={{-8,43},{-8,
            -24},{-46,-24},{-46,-44},{-26,-44}}, color={0,0,127}));
    connect(gainRhoCp.y, divFlow.u2) annotation (Line(points={{-3,-44},{0,
            -44},{0,-24},{2,-24},{2,-16}}, color={0,0,127}));
    connect(limitFlow.y, qvRef) annotation (Line(points={{99,-2},{120,-2},
            {120,2},{138,2}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
  end volumeFlowControlHeatingSystem;
end Control;
