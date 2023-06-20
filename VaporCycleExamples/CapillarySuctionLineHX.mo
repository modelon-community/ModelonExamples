within VaporCycleExamples;

model CapillarySuctionLineHX
  extends .Modelon.Icons.Experiment;
  extends .VaporCycle.Templates.ExperimentWorkingFluid(redeclare replaceable package
              WorkingFluid =
        .Modelon.Media.PreDefined.TwoPhase.R134aSBTLExtendedRange);

  replaceable .Modelica.Blocks.Sources.Ramp h_source(
    height=0.0,
    startTime=1.0e6,
    duration=2,
    final offset=init.h_in) constrainedby .Modelica.Blocks.Interfaces.SO
    "Refrigerant inlet enthalpy" annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=0,
        origin={-170,40})));
  replaceable .Modelica.Blocks.Sources.Ramp mflow_source(
    final offset=init.p_in,
    height=0,
    startTime=1e6,
    duration=2) constrainedby .Modelica.Blocks.Interfaces.SO
    "Refrigerant inlet mass flow rate" annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=0,
        origin={-170,20})));
  .VaporCycle.Sources.TwoPhasePressureSource source(
    use_p_in=true,
    energyInput=.Modelon.ThermoFluid.Choices.EnergyDefinition.h,
    use_Th_in=true,
    redeclare package Medium = WorkingFluid,
    N=1) annotation (Placement(transformation(extent={{-150,0},{-130,20}},
          rotation=0)));
  .VaporCycle.Sources.TwoPhasePressureSource sink(
    redeclare package Medium = WorkingFluid,
    use_p_in=true,
    energyInput=.Modelon.ThermoFluid.Choices.EnergyDefinition.h,
    use_Th_in=true,
    N=1) annotation (Placement(transformation(extent={{20,20},{0,40}},
          rotation=0)));

  .VaporCycle.Sources.TwoPhaseFlowSource source_sec(
    energyInput=.Modelon.ThermoFluid.Choices.EnergyDefinition.h,
    use_m_flow_in=true,
    use_Th_in=true,
    redeclare package Medium = WorkingFluid,
    p_start=init.p_in_sec) annotation (Placement(transformation(extent={{20,50},
            {0,70}},  rotation=0)));
  .VaporCycle.Sources.TwoPhasePressureSource sink_sec(
    energyInput=.Modelon.ThermoFluid.Choices.EnergyDefinition.h,
    use_p_in=true,
    use_Th_in=true,
    redeclare package Medium = WorkingFluid,
    m_flow_start=fill(init.mflow_sec, sink_sec.source.N),
    N=1)                                                  annotation (Placement(
        transformation(extent={{-110,50},{-90,70}},rotation=0)));

  replaceable .Modelica.Blocks.Sources.Ramp h_sink(
    height=0.0,
    startTime=1.0e6,
    duration=2,
    offset=init.h_out) constrainedby .Modelica.Blocks.Interfaces.SO
    "Refrigerant sink enthalpy" annotation (Placement(transformation(
        origin={60,40},
        extent={{-5,5},{5,-5}},
        rotation=180)));

  replaceable
  .VaporCycle.HeatExchangers.TwoPhaseTwoPhase.CapillarySuctionLine slhx(
    initOpt=init.initOpt,
    n_HX=10,
    n_capIn=10,
    n_capOut=10,
    CF_Friction=1.18,
    includeAcceleration=false,
    kineticEnergyInBalance=false,
    massLessWall=true,
    useMeanTempDrivenQ=true,
    redeclare package PrimaryWorkingFluid = WorkingFluid,
    redeclare package SecondaryWorkingFluid = WorkingFluid,
    init_sec(
      p_in=init.p_in_sec,
      dp=init.p_in_sec - init.p_out_sec,
      p_out=init.p_out_sec,
      m_flow=init.mflow_sec,
      initFromEnthalpy=true,
      T_in=268.15,
      T_out=295.15,
      h_in=init.h_in_sec,
      h_out=init.h_in_sec),
    init_prim(
      p_in=init.p_in,
      dp=init.p_in - init.p_out,
      p_out=init.p_out,
      initFromEnthalpy=true,
      T_in=298.15,
      T_out=250.25,
      h_in=init.h_in,
      m_flow=init.mflow,
      h_out=init.h_out),
    use_stabilizer_twoPhaseFraction=true)
    annotation (Placement(transformation(extent={{-61,30},{-19,70}})));

  replaceable .Modelica.Blocks.Sources.Ramp h_source_sec(
    height=0.0,
    startTime=1.0e6,
    duration=2,
    offset=init.h_in_sec) constrainedby .Modelica.Blocks.Interfaces.SO
    "Refrigerant sink enthalpy" annotation (Placement(transformation(
        origin={60,80},
        extent={{-5,5},{5,-5}},
        rotation=180)));
  replaceable .Modelica.Blocks.Sources.Ramp p_sink_sec(
    height=0.0,
    startTime=1.0e6,
    duration=2,
    final offset=init.p_out_init_sec) constrainedby
    .Modelica.Blocks.Interfaces.SO "Refrigerant outlet pressure" annotation (
      Placement(transformation(
        origin={-170,60},
        extent={{-5,5},{5,-5}},
        rotation=0)));
  replaceable .Modelica.Blocks.Sources.Ramp h_sink_sec(
    height=0.0,
    startTime=1.0e6,
    duration=2,
    final offset=init.h_out_sec) constrainedby .Modelica.Blocks.Interfaces.SO
    "Refrigerant inlet enthalpy" annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=0,
        origin={-170,80})));
  replaceable .Modelica.Blocks.Sources.Ramp mflow_source_sec(
    height=0,
    startTime=1e6,
    duration=2,
    final offset=init.mflow_init_sec) constrainedby
    .Modelica.Blocks.Interfaces.SO "Refrigerant inlet mass flow rate"
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=180,
        origin={60,60})));
  //   Modelica.Units.SI.Temperature[slhx.n] T_adiabatic_left;
  //   Modelica.Units.SI.Temperature[slhx.n] T_non_adiabatic;
  //   Modelica.Units.SI.Temperature[slhx.n] T_adiabatic_right;
  //
  //   Modelica.Units.SI.Temperature T_adiabatic_left_in;
  //   Modelica.Units.SI.Temperature T_adiabatic_left_out;
  //   Modelica.Units.SI.Temperature T_non_adiabatic_in;
  //   Modelica.Units.SI.Temperature T_non_adiabatic_out;
  //   Modelica.Units.SI.Temperature T_adiabatic_right_in;
  //   Modelica.Units.SI.Temperature T_adiabatic_right_out;
  //   Modelica.Units.SI.Length[slhx.n] L_adiabatic_left;
  //   Modelica.Units.SI.Length[slhx.n] L_non_adiabatic;
  //   Modelica.Units.SI.Length[slhx.n] L_adiabatic_right;

  .VaporCycle.HeatExchangers.TwoPhaseTwoPhase.Experiments.TestbenchInit init(
    initType=3,
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.initialValues,
    mflow=0.00151,
    p_in=900000,
    p_out=116900,
    h_in=234.55e3,
    h_out=207.740e3,
    mflow_sec=0.00151,
    p_in_sec=157430,
    p_out_sec=157070,
    h_in_sec=398.14e3,
    h_out_sec=411293,
    p_in_init=slhx.portA_prim.p,
    dT_sat_init=10,
    p_in_init_sec=slhx.portA_sec.p,
    dT_sat_init_sec=1)
    annotation (Placement(transformation(extent={{-220,60},{-200,80}})));
  .Modelica.Blocks.Continuous.LimPID PID(
    controllerType=.Modelica.Blocks.Types.SimpleController.PI,
    k=0.1,
    Ti=2,
    yMax=9e5,
    yMin=0.1e5,
    initType=.Modelica.Blocks.Types.Init.InitialOutput,
    y_start=init.p_out)
    annotation (Placement(transformation(extent={{47,3},{33,17}})));
  .Modelica.Blocks.Sources.Constant inputMflowrate(k=init.mflow)
    annotation (Placement(transformation(extent={{86,4},{74,16}})));
  .VaporCycle.Sensors.FlowSensor flowSensor(redeclare package Medium =
        WorkingFluid) annotation (Placement(transformation(
        extent={{10,-12},{-10,12}},
        rotation=180,
        origin={-90,12})));
  .VaporCycle.FlowResistances.GenericFlowResistance flowResistance(
    redeclare package Medium = WorkingFluid,
    redeclare model Friction =
        .Modelon.ThermoFluid.FlowResistances.FrictionModels.QuadraticOperatingPointLoss (
        d0(displayUnit="kg/m3") = 1000,
        dp0(displayUnit="Pa") = 1,
        m_flow0=1),
    T_start=323.15,
    mflow_start=init.mflow)
    annotation (Placement(transformation(extent={{-126,3},{-114,17}})));

  replaceable .VaporCycle.Utilities.Visualizers.pH_Diagrams.pH_R134a_2lines ph_diagram(
    x=cat(
        1,
        {slhx.capillaryInlet.pipe.hA},
        slhx.capillaryInlet.pipe.h,
        {slhx.capillaryInlet.pipe.h[end]},
        {slhx.capillarySLHX.pipe.hA},
        slhx.capillarySLHX.pipe.h,
        {slhx.capillarySLHX.pipe.h[end]},
        {slhx.capillaryOutlet.pipe.hA},
        slhx.capillaryOutlet.pipe.h,
        {slhx.capillaryOutlet.pipe.h[end]}),
    y=.Modelica.Math.log(cat(
        1,
        {slhx.capillaryInlet.portA.p},
        slhx.capillaryInlet.pipe.p,
        {slhx.capillaryInlet.portB.p},
        {slhx.capillarySLHX.portA.p},
        slhx.capillarySLHX.pipe.p,
        {slhx.capillarySLHX.portB.p},
        {slhx.capillaryOutlet.portA.p},
        slhx.capillaryOutlet.pipe.p,
        {slhx.capillaryOutlet.portB.p})),
    x2=cat(
        1,
        {slhx.suctionLineInlet.pipe.hA},
        slhx.suctionLineInlet.pipe.h,
        {slhx.suctionLineInlet.pipe.h[end]},
        {slhx.suctionLineSLHX.pipe.hA},
        slhx.suctionLineSLHX.pipe.h,
        {slhx.suctionLineSLHX.pipe.h[end]},
        {slhx.suctionLineOutlet.pipe.hA},
        slhx.suctionLineOutlet.pipe.h,
        {slhx.suctionLineOutlet.pipe.h[end]}),
    y2=.Modelica.Math.log(cat(
        1,
        {slhx.suctionLineInlet.portA.p},
        slhx.suctionLineInlet.pipe.p,
        {slhx.suctionLineInlet.portB.p},
        {slhx.suctionLineSLHX.portA.p},
        slhx.suctionLineSLHX.pipe.p,
        {slhx.suctionLineSLHX.portB.p},
        {slhx.suctionLineOutlet.portA.p},
        slhx.suctionLineOutlet.pipe.p,
        {slhx.suctionLineOutlet.portB.p})),
    color2={255,0,0})
    constrainedby .Modelon.Visualizers.XYDiagramBackgroundImage2lines
    annotation (choicesAllMatching=true,
      Placement(transformation(
        extent={{149.50123288297738,-18.030017117022624},{270.4987671170226,102.96751711702262}},
        rotation=0.0,
        origin={-34,-6})));

  .Modelon.Visualizers.XYDiagram2 xYDiagram2_2(
    title="Temperature profile",
    xLabel="Along the tube length",
    yLabel="Temperature in K",
    maxX=4,
    minY=200,
    maxY=350,
    x1=L_cap_plot,
    y1=T_cap_plot,
    color1={0,0,255},
    x2=L_suc_plot,
    y2=T_suc_plot,
    color2={255,0,0}) annotation (Placement(transformation(
        extent={{35.33333333333334,-120.0},{115.33333333333334,-40.0}},
        rotation=0.0,
        origin={-76,-8})));

  .Modelica.Units.SI.Temperature T_cap_plot[:]=cat(
      1,
      {slhx.capillaryInlet.summary.T_in},
      slhx.capillaryInlet.pipe.T,
      slhx.capillarySLHX.pipe.T,
      slhx.capillaryOutlet.pipe.T);

  .Modelica.Units.SI.Temperature T_suc_plot[:]=cat(
      1,
      {slhx.suctionLineInlet.summary.T_in},
      slhx.suctionLineInlet.pipe.T,
      slhx.suctionLineSLHX.pipe.T,
      slhx.suctionLineOutlet.pipe.T);

  .Modelica.Units.SI.Pressure P_cap_plot[:]=cat(
      1,
      {slhx.summary.p_in},
      slhx.capillaryInlet.pipe.p,
      slhx.capillarySLHX.pipe.p,
      slhx.capillaryOutlet.pipe.p);

  .Modelica.Units.SI.Pressure P_suc_plot[:]=cat(
      1,
      {slhx.summary.p_sec_in},
      slhx.suctionLineInlet.pipe.p,
      slhx.suctionLineSLHX.pipe.p,
      slhx.suctionLineOutlet.pipe.p);
  .Modelica.Units.SI.Length L_cap_plot[:]=linspace(
      0,
      slhx.geometry.capInLength + slhx.geometry.heatExchangeLength + slhx.geometry.capOutLength,
      (slhx.n_capIn + slhx.n_HX + slhx.n_capOut) + 1);

  .Modelica.Units.SI.Length L_suc_plot[:]=linspace(
      0,
      slhx.geometry.suctionLengthIn + slhx.geometry.heatExchangeLength + slhx.geometry.suctionLengthOut,
      (slhx.n_sucIn + slhx.n_HX + slhx.n_sucOut) + 1);

  .Modelica.Units.SI.Temperature T_cap_hx_plot[:]=cat(
      1,
      {slhx.capillarySLHX.summary.T_in},
      slhx.capillarySLHX.pipe.T);

  .Modelica.Units.SI.Temperature T_suc_hx_plot[:]=cat(
      1,
      {slhx.suctionLineSLHX.pipe.T[slhx.n_HX - i] for i in 0:slhx.n_HX - 1},
      {slhx.suctionLineSLHX.summary.T_in});

  .Modelica.Units.SI.Length L_hx_plot[:]=linspace(
      0,
      slhx.geometry.heatExchangeLength,
      slhx.n_HX + 1);

  .Modelon.Visualizers.XYDiagram2 xYDiagram2_1(
    title="Temperature profile for HX region",
    xLabel="Along the tube hx length",
    yLabel="Temperature in K",
    maxX=1.6,
    minY=200,
    maxY=350,
    x1=L_hx_plot,
    y1=T_cap_hx_plot,
    color1={0,0,255},
    x2=L_hx_plot,
    y2=T_suc_hx_plot,
    color2={255,0,0}) annotation (Placement(transformation(
        extent={{180.0,-120.0},{260.0,-40.0}},
        rotation=0.0,
        origin={-76,-8})));
  .Modelon.Visualizers.XYDiagram2 xYDiagram2_3(
    title="Pressure profile",
    xLabel="Along the tube length",
    yLabel="Pressure in bar",
    maxX=4,
    minY=1,
    maxY=9,
    x1=L_cap_plot,
    y1=P_cap_plot/1e5,
    color1={0,0,255},
    x2=L_suc_plot,
    y2=P_suc_plot/1e5,
    color2={255,0,0}) annotation (Placement(transformation(
        extent={{-100.0,-120.0},{-20.0,-40.0}},
        rotation=0.0,
        origin={-76,-8})));
  inner .VaporCycle.AggregateTwoPhaseProperties aggregateTwoPhaseProperties
    annotation (Placement(transformation(extent={{-240,-140},{-220,-120}})));
equation

  //   for i in 1:slhx.n loop
  //
  //     T_adiabatic_left[i] = slhx.adiabaticCapTubeLeft.pipe.T[i];
  //     T_non_adiabatic[i] = slhx.nonAdiabaticCapTube.pipe.T[i];
  //     T_adiabatic_right[i] = slhx.adiabaticCapTubeRight.pipe.T[i];
  //
  //     L_adiabatic_left[i] = slhx.adiabaticCapTubeLeft.pipe.L[i];
  //     L_non_adiabatic[i] = slhx.nonAdiabaticCapTube.pipe.L[i];
  //     L_adiabatic_right[i] = slhx.adiabaticCapTubeRight.pipe.L[i];
  //
  //   end for;
  //
  //   T_adiabatic_left_in = slhx.adiabaticCapTubeLeft.summary.T_in;
  //   T_adiabatic_left_out = slhx.adiabaticCapTubeLeft.summary.T_out;
  //   T_non_adiabatic_in = slhx.nonAdiabaticCapTube.summary.T_in;
  //   T_non_adiabatic_out = slhx.nonAdiabaticCapTube.summary.T_out;
  //   T_adiabatic_right_in = slhx.adiabaticCapTubeRight.summary.T_in;
  //   T_adiabatic_right_out = slhx.adiabaticCapTubeRight.summary.T_out;

  connect(source_sec.port, slhx.portA_sec) annotation (Line(
      points={{2.2,60},{-8,60},{-8,62},{-19,62}},
      color={0,190,0},
      smooth=Smooth.None));
  connect(sink_sec.p_in, p_sink_sec.y) annotation (Line(points={{-105,65},{-140,
          65},{-140,60},{-164.5,60}},             color={0,0,127}));
  connect(inputMflowrate.y, PID.u_s)
    annotation (Line(points={{73.4,10},{48.4,10}},
                                                 color={0,0,127}));
  connect(source.port[1], flowResistance.portA)
    annotation (Line(points={{-132,10},{-126,10}},
                                                 color={0,190,0}));
  connect(flowResistance.portB, flowSensor.portA)
    annotation (Line(points={{-114,10},{-100,10}},
                                                 color={0,190,0}));
  connect(flowSensor.m_flow, PID.u_m) annotation (Line(points={{-90,0},{-90,-6},
          {40,-6},{40,1.6}},color={0,0,127}));
  connect(sink.port[1], slhx.portB_prim) annotation (Line(points={{2,30},{-8,30},
          {-8,38},{-19,38}},      color={0,190,0}));
  connect(PID.y, sink.p_in) annotation (Line(points={{32.3,10},{28,10},{28,35},
          {15,35}}, color={0,0,127}));
  connect(h_sink_sec.y, sink_sec.h_in)
    annotation (Line(points={{-164.5,80},{-100,80},{-100,67}},
                                                            color={0,0,127}));
  connect(sink.h_in, h_sink.y)
    annotation (Line(points={{10,37},{10,44},{40,44},{40,40},{54.5,40}},
                                                         color={0,0,127}));
  connect(h_source.y, source.h_in)
    annotation (Line(points={{-164.5,40},{-140,40},{-140,17}},
                                                            color={0,0,127}));
  connect(mflow_source.y, source.p_in)
    annotation (Line(points={{-164.5,20},{-145,20},{-145,15}},
                                                            color={0,0,127}));
  connect(source_sec.m_flow_in, mflow_source_sec.y) annotation (Line(points={{
          15.2,65},{15.2,72},{40,72},{40,60},{54.5,60}}, color={0,0,127}));
  connect(source_sec.h_in, h_source_sec.y)
    annotation (Line(points={{11.2,67},{11.2,80},{54.5,80}}, color={0,0,127}));
  connect(flowSensor.portB, slhx.portA_prim) annotation (Line(points={{-80,10},
          {-72,10},{-72,38},{-61,38}}, color={0,190,0}));
  connect(sink_sec.port[1], slhx.portB_sec) annotation (Line(points={{-92,60},{
          -80,60},{-80,62},{-61,62}}, color={0,190,0}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-240,-140},{
            240,100}})),
    Documentation(revisions="<html>
Copyright &copy; 2004-2023, MODELON AB <br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br />This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br />or by other means have their origin from any Modelon Library.
</html>", info="<html>
<h4>Capillary Suction Line</h4>
<p>This experiment is for testing <a href=\"modelica://VaporCycle.HeatExchangers.TwoPhaseTwoPhase.CapillarySuctionLine\">the capillary tube with a suction line heat exchanger</a>.</p>
<p>The capillary tube and suction line are parameterized based on the measured data mentioned in the reference section. As per the reference, the inlet pressure and inlet mass flow rate are known. So, provided the inlet mass flow rate at the inlet of the capillary tube and introduced the PID controller to identify the outlet pressure to match the inlet pressure of the capillary. The visualizer &quot;Temperature profile&quot; shows the general temperature profile of the liquid and two-phase flow through the capillary tube along the length (blue color) and also it clearly shows the inlet , the heat exchanger region and oulet region of the capillary tube. It also shows the temperature profile of the suction line (red color). There is a PH diagram to show the enthalpy vs pressure of the capillary tube in blue color respresentation and enthalpy vs pressure of the suction line in black color representation</p>
<h4>Capillary Tube Test Results</h4>
<p><br><img src=\"modelica://VaporCycle/Resources/Images/CapillaryTubeSuctionLineTestResult.png\"/> </p>
<h4>References</h4>
<p>[1] Lyun-Su Kim &bull; Ki-dong Son &bull; Debasish Sarker &bull; Ji Hwan Jeong &bull; Sung Hong Lee: </p>
<p>An assessment of models for predicting refrigerant characteristics in adiabatic and non-adiabatic capillary tubes </p>
<p>In Springer-Verlag 2010, Heat Mass Transfer (2011) 47:163&ndash;180 </p>
<p> <a href=\"https://doi.org/10.1007/s00231-010-0697-0\">DOI: 10.1007/s00231-010-0697-0</a> </p>
</html>"),
    experiment(
      StopTime=10,
      Tolerance=1e-07,
      __Dymola_Algorithm="Dassl"),
    Diagram(coordinateSystem(extent={{-240,-140},{240,120}})),
    Icon(coordinateSystem(extent={{-100,-100},{100,100}})));
end CapillarySuctionLineHX;
