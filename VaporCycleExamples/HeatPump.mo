within VaporCycleExamples;

model HeatPump "R-407c heat pump"
  extends .Modelon.Icons.Experiment;
  extends .VaporCycle.Templates.ExperimentLiquidFluid;
  extends .VaporCycle.Templates.ExperimentWorkingFluid(
      redeclare replaceable package WorkingFluid =
        .VaporCycle.Media.Hydrofluorocarbons.R407cPseudoPure);

  inner .Modelon.ThermoFluid.Settings_TF settings_TF(usePbS = false) annotation(Placement(transformation(extent = {{0.0,100.0},{20.0,120.0}},rotation = 0.0,origin = {0.0,0.0})));

  .VaporCycle.Experiments.RefrigerationSystem.VaporCompressionCycle summary(
    P_compressor=compressor.summary.shaftPower,
    p_high=compressor.summary.p_dis,
    p_low=compressor.summary.p_suc,
    h_evap_out=evaporator.summary.h_out,
    h_evap_in=evaporator.summary.h_in,
    h_cond_out=condenser.summary.h_out,
    h_cond_in=condenser.summary.h_in,
    m_flow=expansionValve.summary.m_flow,
    dp_evap=evaporator.summary.dp,
    dp_cond=condenser.summary.dp,
    recLevel=receiver.summary.relativeLevel,
    recQuality=receiver.summary.quality,
    subcooling=condenser.summary.dT_sat,
    superheat=evaporator.summary.dT_sat,
    COP=summary.P_condenser/max(1e-5, summary.P_compressor),
    M=aggregateTwoPhaseProperties.M,
    V=aggregateTwoPhaseProperties.V,
    P_evaporator=evaporator.summary.Q_flow,
    P_condenser=condenser.hex.secondaryChannel.Q_tot,
    T_evap_in_sec=evaporator.summary.T_sec_in,
    T_evap_out_sec=evaporator.summary.T_sec_out,
    T_cond_in_sec=condenser.summary.T_sec_in,
    T_cond_out_sec=condenser.summary.T_sec_out)
    annotation (Placement(transformation(extent={{-40,100},{-20,120}})));
  .VaporCycle.Experiments.RefrigerationSystem.VaporCompressionCycleInit init(
    redeclare package WorkingFluid = WorkingFluid,
    T_sc=12,
    T_sh=4,
    p_receiver=receiver.summary.p_in,
    M_receiver=receiver.summary.M,
    V_receiver=receiver.summary.V,
    V_workingFluid=summary.V,
    charge=summary.specificCharge,
    phi_evap_sec=0.5,
    medium_cond=2,
    medium_evap=2,
    mflow_start=0.05,
    mflow_cond_sec=0.2,
    mflow_evap_sec=0.25,
    compSpeed=100,
    charge_init=360,
    initType=2,
    p_high=2450000,
    dp_high=10000,
    p_suction=600000,
    dp_low=10000,
    dp_pipe=2000,
    T_cond_sec=311.15,
    T_evap_sec=283.15)
    annotation (Placement(transformation(extent={{-80.0,100.0},{-60.0,120.0}},rotation = 0.0,origin = {0.0,0.0})));
  .VaporCycle.HeatExchangers.TwoPhaseLiquid.Examples.Chiller condenser(
    init_liq(
      T_in=init.T_cond_sec,
      m_flow=init.mflow_cond_sec,
      T_out=init.T_cond_sec + 30,
      p_in=110000,
      p_out=100000),
    init_wf(
      p_in=init.p_cond_in,
      p_out=init.p_cond_out,
      h_in=init.h_cond_in,
      h_out=init.h_cond_out,
      m_flow=init.mflow_start),
    redeclare package WorkingFluid = WorkingFluid,
    hx_type=1,
    n=5,
    redeclare replaceable model Friction =
        .VaporCycle.Pipes.SubComponents.FlowResistance.TwoPhase.DensityProfileFriction
        (
        h0_in=300e3,
        h0_out=130e3,
        p0_in=2400000,
        p0_out=2390000,
        mflow0=0.06),
    redeclare model HeatTransfer_liq =
        .VaporCycle.Pipes.SubComponents.HeatTransfer.SinglePhase.DittusBoelterAdjustable,
    Ah_liq_geo=2,
    A_wfl_geo=5e-4,
    Ah_wfl_geo=2,
    L_liq_geo=1.5,
    L_wfl_geo=1.5,
    wt_start=0.5,
    redeclare model HeatTransfer =
        .VaporCycle.Pipes.SubComponents.HeatTransfer.TwoPhase.CondensationShah,
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.initialValues,
    redeclare package Liquid = Liquid) annotation (Placement(transformation(extent={{-2,38},{-22,58}})));

  .VaporCycle.HeatExchangers.TwoPhaseLiquid.Examples.Chiller evaporator(
    redeclare package WorkingFluid = WorkingFluid,
    init_liq(
      T_in=init.T_evap_sec,
      m_flow=init.mflow_evap_sec,
      T_out=init.T_evap_sec - 10),
    init_wf(
      p_in=init.p_evap_in,
      p_out=init.p_evap_out,
      h_in=init.h_cond_out,
      h_out=init.h_suction,
      m_flow=init.mflow_start),
    n=5,
    redeclare model HeatTransfer =
        .VaporCycle.Pipes.SubComponents.HeatTransfer.TwoPhase.EvaporationGungorWinterton,
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.initialValues,
    redeclare replaceable model Friction =
        .VaporCycle.Pipes.SubComponents.FlowResistance.TwoPhase.DensityProfileFriction
        (
        h0_in=140e3,
        h0_out=270e3,
        mflow0=0.06,
        p0_in=405000,
        p0_out=400000),
    L_liq_geo=1.5,
    L_wfl_geo=1.5,
    Ah_liq_geo=3,
    Ah_wfl_geo=1,
    redeclare model HeatTransfer_liq =
        .VaporCycle.Pipes.SubComponents.HeatTransfer.SinglePhase.ConstantCoefficient
        (                                                                                                           alpha=5000),
    redeclare package Liquid = Liquid) annotation (Placement(transformation(extent={{-42,-92},{-22,-72}})));

  .VaporCycle.Compressors.FixedDisplacementTableBased  compressor(redeclare package Medium =
        WorkingFluid,
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.initialValues,
    initFromEnthalpy=true,
    p_start=init.p_suction,
    h_start=init.h_suction,
    tableOnFile=false,
    etaMechMap=[0.0,1,10; 10,0.97,0.97; 20,0.97,0.97],
    volEffMap=[0.0,1,10; 10,0.95,0.95; 20,0.95,0.95],
    etaIsMap=[0.0,1,10; 10,0.7,0.7; 20,0.7,0.7],
    V_MaxDisplacement=2e-5)                           annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={32,-20})));
  .VaporCycle.Pipes.PipeAdiabatic pipe1(
    redeclare package Medium = WorkingFluid,
    n=1,
    n_channels=1,
    L=0.5,
    D=0.02,
    initFromEnthalpy=true,
    m_flow_start=init.mflow_start,
    redeclare model Friction =
        .VaporCycle.Pipes.SubComponents.FlowResistance.TwoPhase.SimpleFromDensity
        (mflow0=init.mflow_start),
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    h_in_start=init.h_cond_in,
    h_out_start=init.h_cond_in,
    p_in_start=init.p_high,
    p_out_start=init.p_high - init.dp_pipe)
                           annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={32,16})));
  .VaporCycle.Pipes.PipeAdiabatic pipe2(redeclare package Medium = WorkingFluid,
    n=1,
    n_channels=1,
    L=0.3,
    D=0.008,
    initFromEnthalpy=true,
    m_flow_start=init.mflow_start,
    redeclare model Friction =
        .VaporCycle.Pipes.SubComponents.FlowResistance.TwoPhase.SimpleFromDensity
        (mflow0=init.mflow_start),
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    p_in_start=init.p_cond_out,
    p_out_start=init.p_cond_out - init.dp_pipe,
    h_in_start=init.h_cond_out,
    h_out_start=init.h_cond_out,
    dp_smooth=100)
    annotation (Placement(transformation(extent={{10.0,-10.0},{-10.0,10.0}},rotation = 90.0,origin = {-86.0,26.0})));
  .VaporCycle.Tanks.LiquidReceiver receiver(redeclare package Medium = WorkingFluid,
    H=0.15,
    H_suction=0.01,
    desiccant=false,
    initFromEnthalpy=true,
    zeta=200,
    staticHead=false,
    p_start=init.p_cond_out - init.dp_pipe,
    h_start=init.h_cond_out,
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    D=0.05528)
    annotation (Placement(transformation(extent={{-80.0,-20.0},{-100.0,0.0}},rotation = 0.0,origin = {0.0,0.0})));
  .VaporCycle.Pipes.PipeAdiabatic pipe3(redeclare package Medium = WorkingFluid,
    n=1,
    n_channels=1,
    L=0.3,
    initFromEnthalpy=true,
    m_flow_start=init.mflow_start,
    redeclare model Friction =
        .VaporCycle.Pipes.SubComponents.FlowResistance.TwoPhase.SimpleFromDensity
        (mflow0=init.mflow_start),
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    p_in_start=init.p_cond_out - init.dp_pipe - 0.1*init.dp_pipe,
    p_out_start=init.p_cond_out - 2*init.dp_pipe,
    h_in_start=init.h_cond_out,
    h_out_start=init.h_cond_out,
    D=0.008)                annotation (Placement(transformation(
        extent={{-10.0,-10.0},{10.0,10.0}},
        rotation=-90.0,
        origin={-110.0,-30.0})));

  .VaporCycle.Valves.SimpleTXV expansionValve(redeclare package Medium = WorkingFluid,
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.initialValues,
    mflow_start=init.mflow_start,
    yMax=0.12,
    yMin=0.001,
    p_start=init.p_cond_out - 2.1*init.dp_pipe,
    h_start=init.h_cond_out,
    V=1e-06)                 annotation (Placement(transformation(
        extent={{-10.0,10.0},{10.0,-10.0}},
        rotation=-90.0,
        origin={-110.0,-60.0})));
  .VaporCycle.Sources.LiquidFlowSource
                        liqIn_cond(
    use_T_in=true,
    flowDefinition=.Modelon.ThermoFluid.Choices.FlowDefinition.m_flow,
    use_flow_in=true,
    redeclare package Medium = Liquid,
    p_start=1.2e5)
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  .VaporCycle.Sources.LiquidPressureSource
                            liqOut_cond(
    redeclare package Medium = Liquid,
    N=1,
    p=101300,
    m_flow_start=fill(init.mflow_cond_sec, liqOut_cond.pressureBoundary.N))
                                             annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={26,62})));
  .VaporCycle.Sources.LiquidPressureSource
                            liqOut_evap(
    redeclare package Medium = Liquid,
    N=1,
    p=101300,
    m_flow_start=fill(init.mflow_evap_sec, liqOut_evap.pressureBoundary.N))
                                             annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-56,-68})));
  .VaporCycle.Sources.LiquidFlowSource
                        liqIn_evap(
    use_T_in=true,
    use_flow_in=true,
    flowDefinition=.Modelon.ThermoFluid.Choices.FlowDefinition.m_flow,
    temperatureUnit=.Modelon.ThermoFluid.Choices.RealTemperatureUnit.K,
    redeclare package Medium = Liquid,
    p_start=1.2e5)
    annotation (Placement(transformation(extent={{2,-86},{-18,-66}})));
  .Modelon.Visualizers.RealValue valueSuperheat(precision=1, number=
        summary.superheat)
    annotation (Placement(transformation(extent={{128,-90},{148,-70}})));
  .Modelon.Visualizers.RealValue valueTime(precision=1, number=time)
    annotation (Placement(transformation(extent={{128,-112},{148,-92}})));
  .Modelon.Visualizers.RealValue valueP_evap(precision=0, number=
        summary.P_condenser)
    annotation (Placement(transformation(extent={{164,-90},{184,-70}})));
  .Modelon.Visualizers.RealValue valueCOP(precision=1, number=
        summary.COP)
    annotation (Placement(transformation(extent={{164,-112},{184,-92}})));
  .VaporCycle.Sources.Speed rps
    annotation (Placement(transformation(extent={{68,-30},{48,-10}})));
  .Modelica.Blocks.Sources.Ramp T_cond(
    offset=init.T_cond_sec,
    duration=5,
    startTime=300,
    height=10)
    annotation (Placement(transformation(extent={{-5,-5},{5,5}},
        rotation=270,
        origin={-49,81})));
  .Modelica.Blocks.Sources.Ramp mflow_cond(
    duration=2,
    offset=init.mflow_cond_sec,
    startTime=450,
    height=0.1)
    annotation (Placement(transformation(extent={{-92,60},{-82,70}})));
  .Modelica.Blocks.Sources.Ramp T_evap(
    offset=init.T_evap_sec,
    duration=5,
    height=5,
    startTime=1e6)
    annotation (Placement(transformation(extent={{-40,-68},{-28,-56}})));
  .Modelica.Blocks.Sources.Ramp mflow_evap(
    duration=1,
    startTime=60,
    height=0,
    offset=init.mflow_evap_sec)
    annotation (Placement(transformation(extent={{-40,-48},{-28,-36}})));
  .Modelica.Blocks.Sources.Ramp speed(
    offset=init.compSpeed,
    duration=5,
    height=-10,
    startTime=1e6)
    annotation (Placement(transformation(extent={{96,-26},{84,-14}})));

  .VaporCycle.Pipes.PipeAdiabatic pipe4(
    redeclare package Medium = WorkingFluid,
    n=1,
    n_channels=1,
    L=0.4,
    D=0.015,
    initFromEnthalpy=true,
    m_flow_start=init.mflow_start,
    redeclare model Friction =
        .VaporCycle.Pipes.SubComponents.FlowResistance.TwoPhase.SimpleFromDensity
        (dp0=10000, mflow0=init.mflow_start),
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    p_in_start=init.p_suction + init.dp_pipe,
    p_out_start=init.p_suction,
    h_in_start=init.h_suction,
    h_out_start=init.h_suction)               annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={32,-50})));
  .VaporCycle.Sensors.SubCoolingSensor subCoolingSensor(
    redeclare package Medium = WorkingFluid,
    allowNegative = true) annotation(Placement(transformation(extent = {{-10.0,-12.0},{10.0,12.0}},origin = {-70.0,44.0},rotation = -180.0)));
protected
  parameter .Modelica.Units.SI.Time startTime(fixed=false);
public
  .VaporCycle.Sensors.SuperHeatSensor superHeatSensor(redeclare package Medium =
        WorkingFluid)
    annotation (Placement(transformation(extent={{30,-74},{50,-98}})));
  inner .VaporCycle.AggregateTwoPhaseProperties aggregateTwoPhaseProperties
    annotation (Placement(transformation(extent={{-120,100},{-100,120}})));
  .VaporCycle.Utilities.Visualizers.pH_Diagrams.PH_R407c pH_R407c(x={summary.h_cond_in,
        summary.h_cond_out,summary.h_evap_in,summary.h_evap_out,summary.h_cond_in},
      y=.Modelica.Math.log({condenser.summary.p_in,condenser.summary.p_out,
        evaporator.summary.p_in,evaporator.summary.p_out,condenser.summary.p_in}))
    annotation (Placement(transformation(extent={{46.0,-24.0},{194.0,124.0}},rotation = 0.0,origin = {0.0,0.0})));
  .VaporCycle.Sensors.MultiDisplaySensor multiDisplaySensor(redeclare package Medium =
        WorkingFluid)
    annotation (Placement(transformation(extent={{0,-98},{20,-78}})));
  .VaporCycle.Sensors.MultiDisplaySensor multiDisplaySensor1(redeclare package Medium =
        WorkingFluid)
    annotation (Placement(transformation(extent={{26,52},{6,32}})));
  .VaporCycle.Sensors.MultiDisplaySensor multiDisplaySensor2(redeclare package Medium =
        WorkingFluid)
    annotation (Placement(transformation(extent={{-26,52},{-46,32}})));
  .VaporCycle.Sensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot(massFlowType=.Modelon.ThermoFluid.Choices.MassFlowType.g_per_s,
      displayUnits=true)
    annotation (Placement(transformation(extent={{-48,14},{-24,38}})));
  .VaporCycle.Sensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot1(massFlowType=.Modelon.ThermoFluid.Choices.MassFlowType.g_per_s,
      displayUnits=true)
    annotation (Placement(transformation(extent={{4,14},{28,38}})));
  .VaporCycle.Sensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot2(massFlowType=.Modelon.ThermoFluid.Choices.MassFlowType.g_per_s,
      displayUnits=true)
    annotation (Placement(transformation(extent={{2,-84},{26,-60}})));
initial equation
  startTime=time;
equation
  if not settings_TF.usePbS then
    when sample(startTime,.Modelica.Constants.inf) and init.initType==1 then
      reinit(receiver.separator.volume.h,init.h_receiver);
    end when;
  end if;
  connect(compressor.flange, rps.flange) annotation (Line(
      points={{41,-20},{48,-20}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(T_cond.y,liqIn_cond. T_in)    annotation (Line(
      points={{-49,75.5},{-50,75.5},{-50,67}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(T_evap.y,liqIn_evap. T_in)    annotation (Line(
      points={{-27.4,-62},{-8,-62},{-8,-69}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pipe4.portB, compressor.portA) annotation (Line(
      points={{32,-40},{32,-30}},
      color={0,190,0},
      smooth=Smooth.None));
  connect(compressor.portB, pipe1.portA)
                                        annotation (Line(
      points={{32,-10},{32,6}},
      color={0,190,0},
      smooth=Smooth.None));
  connect(pipe2.portB, receiver.portA) annotation (Line(
      points={{-86,16},{-86,0}},
      color={0,190,0},
      smooth=Smooth.None));
  connect(receiver.portB,pipe3. portA) annotation (Line(
      points={{-94,0},{-110,0},{-110,-20}},
      color={0,190,0},
      smooth=Smooth.None));
  connect(pipe3.portB, expansionValve.portA) annotation (Line(
      points={{-110,-40},{-110,-52}},
      color={0,190,0},
      smooth=Smooth.None));
  connect(mflow_cond.y,liqIn_cond. m_flow_in) annotation (Line(
      points={{-81.5,65},{-54.8,65}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(mflow_evap.y,liqIn_evap. m_flow_in) annotation (Line(
      points={{-27.4,-42},{-3.2,-42},{-3.2,-71}},
      color={0,0,127},
      smooth=Smooth.None));

  connect(rps.inPort, speed.y) annotation (Line(
      points={{70,-20},{83.4,-20}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(superHeatSensor.portB, pipe4.portA) annotation (Line(
      points={{50,-88},{50,-60},{32,-60}},
      color={0,190,0},
      smooth=Smooth.None));
  connect(expansionValve.DeltaT_SH, superHeatSensor.y) annotation (Line(
      points={{-117.5,-56.5},{-128,-56.5},{-128,-106},{40,-106},{40,-97}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(liqIn_cond.port, condenser.portA_sec) annotation (Line(points={{-42,60.4},{-32,60.4},{-32,54},{-22,54}}, color={0,0,255}));
  connect(condenser.portB_sec, liqOut_cond.port[1]) annotation (Line(points={{-2,54},{8,54},{8,62},{18,62}}, color={0,0,255}));
  connect(liqOut_evap.port[1], evaporator.portB_sec) annotation (Line(points={{-56,-76},{-42,-76}}, color={0,0,255}));
  connect(evaporator.portA_sec, liqIn_evap.port) annotation (Line(points={{-22,-76},{-20,-76},{-20,-75.6},{-16,-75.6}}, color={0,0,255}));
  connect(expansionValve.portB, evaporator.portA_prim) annotation (Line(points={{-110,-68},{-110,-88},{-42,-88}}, color={0,190,0}));
  connect(evaporator.portB_prim, multiDisplaySensor.portA) annotation (Line(
        points={{-22,-88},{-10,-88},{-10,-88},{0,-88}}, color={0,190,0}));
  connect(multiDisplaySensor.portB, superHeatSensor.portA)
    annotation (Line(points={{20,-88},{30,-88}}, color={0,190,0}));
  connect(pipe1.portB, multiDisplaySensor1.portA)
    annotation (Line(points={{32,26},{32,42},{26,42}}, color={0,190,0}));
  connect(multiDisplaySensor1.portB, condenser.portA_prim)
    annotation (Line(points={{6,42},{-2,42}}, color={0,190,0}));
  connect(condenser.portB_prim, multiDisplaySensor2.portA)
    annotation (Line(points={{-22,42},{-26,42}}, color={0,190,0}));
  connect(multiDisplaySensor2.u1, multiDisplayVis_phTmdot.y)
    annotation (Line(points={{-36,42},{-36,26}}, color={0,0,0}));
  connect(multiDisplaySensor1.u1, multiDisplayVis_phTmdot1.y)
    annotation (Line(points={{16,42},{16,26}}, color={0,0,0}));
  connect(multiDisplaySensor.u1, multiDisplayVis_phTmdot2.y) annotation (Line(
        points={{10,-88},{14,-88},{14,-72},{14,-72}}, color={0,0,0}));
  connect(pipe2.portA,subCoolingSensor.portB) annotation(Line(points = {{-86,36},{-86,42},{-80,42}},color = {0,190,0}));
  connect(subCoolingSensor.portA,multiDisplaySensor2.portB) annotation(Line(points = {{-60,42},{-46,42}},color = {0,190,0}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
            -120},{200,120}}), graphics={
        Rectangle(
          extent={{120,-60},{200,-120}},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Text(
          extent={{164,-90},{184,-96}},
          lineColor={0,0,127},
          textString="COP",
          fontSize=12),
        Text(
          extent={{156,-66},{190,-72}},
          lineColor={0,0,127},
          fontSize=12,
          textString="Heating power"),
        Text(
          extent={{126,-90},{146,-96}},
          lineColor={0,0,127},
          textString="Time",
          fontSize=12),
        Text(
          extent={{120,-65},{154,-73}},
          lineColor={0,0,127},
          textString="Superheating",
          fontSize=12),
        Text(
          extent={{178,-72},{200,-88}},
          lineColor={0,0,127},
          textString="W",
          fontSize=18),
        Text(
          extent={{142,-72},{164,-88}},
          lineColor={0,0,127},
          fontSize=18,
          textString="K"),
        Text(
          extent={{140,-92},{162,-108}},
          lineColor={0,0,127},
          fontSize=18,
          textString="s")}),               Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})),
    experiment(
      StopTime=800,
      Tolerance=1e-07),
    __Dymola_experimentSetupOutput(equdistant=false),
    Documentation(revisions="<html>
Copyright &copy; 2004-2023, MODELON AB <br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br />This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br />or by other means have their origin from any Modelon Library.
</html>", info="<html>
<p>This is an example of a heat pump system experiment with R-407c as working fluid. A vapor compression cycle with water as secondary fluid in condenser and evaporator transports heat from the low temperature heat source to the high temperature heat sink. The superheating at the evaporator is controlled by the expansion valve. During the simulation, transients are applied to the liquid fluid boundary conditions at both heat exchangers.</p>
<p>Plot the following variables to examine the impact of boundary condition changes:
<ul>
<li>Superheating at the evaporator outlet:<code class=\"modelica\">summary.superheat</code></li>
<li>Subcooling at the condenser outlet:<code class=\"modelica\">summary.subcooling</code></li>
<li>Water outlet temperature at condenser <code class=\"modelica\">summary.T_cond_out_sec</code></li>
<li>Coefficient of performance:<code class=\"modelica\">summary.COP</code></li>
<li>Suction and discharge pressure:<code class=\"modelica\">summary.p_high</code> and <code class=\"modelica\">summary.p_low</code>
<li>Refrigerant mass flow rate:<code class=\"modelica\">summary.m_flow</code></li></ul></p>
</html>"));
end HeatPump;
