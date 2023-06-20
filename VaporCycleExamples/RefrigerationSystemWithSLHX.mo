within VaporCycleExamples;

model RefrigerationSystemWithSLHX
  extends .Modelon.Icons.Experiment;
  extends .VaporCycle.Templates.ExperimentWorkingFluid(redeclare replaceable package WorkingFluid =
        .Modelon.Media.PreDefined.TwoPhase.R134aSBTLExtendedRange);
  extends .VaporCycle.Templates.ExperimentAirMedium;
  //outer Modelon.ThermoFluid.Settings_TF settings_TF "Physics-based solving settings";

  // Ph diagram
  .Modelica.Units.SI.Pressure[:] p_Diagram={compressor.compressor.compressor.portB.p,condenser.summary.p_in,
      condenser.summary.p_out,
      SLHX.summary.p_in,SLHX.summary.p_out,evaporator.summary.p_in,evaporator.summary.p_out,
      SLHX.summary.p_sec_in,SLHX.summary.p_sec_out,compressor.compressor.compressor.portA.p,compressor.compressor.compressor.portB.p}
    "pressures for p-h diagram";
  .Modelica.Units.SI.SpecificEnthalpy[:] h_Diagram={compressor.compressor.compressor.h_out,condenser.summary.h_in,
      condenser.summary.h_out,SLHX.summary.h_in,SLHX.summary.h_out,
      evaporator.summary.h_in,evaporator.summary.h_out,SLHX.summary.h_sec_in,
      SLHX.summary.h_sec_out,
      compressor.compressor.compressor.h_in,compressor.compressor.compressor.h_out}
    "specific enthalpies for p-h diagram";

  //-------------------

  .VaporCycle.HeatExchangers.TwoPhaseAir.WireAndTubeHX condenser(
    T_start_air=init.T_cond_sec,
    redeclare package WorkingFluid = WorkingFluid,
    redeclare package Air = AirMedium,
    init_wf(
      m_flow=init.mflow_start,
      p_in=init.p_cond_in,
      dp=init.dp_high,
      h_in=init.h_cond_in,
      dh=init.h_cond_out - init.h_cond_in),
    n=5,
    redeclare model Friction =
        .VaporCycle.Pipes.SubComponents.FlowResistance.TwoPhase.DensityProfileFriction (
        p0_in=init.p_cond_in,
        p0_out=init.p_cond_out,
        h0_in=init.h_cond_in,
        h0_out=init.h_cond_out,
        mflow0=init.mflow_start),
    wt_start=0.5,
    twoPhaseChannel(use_stabilizer_twoPhaseFraction=true,
        c_pseudo_twoPhaseFraction_const=0.02),redeclare replaceable model HeatTransfer = .VaporCycle.Pipes.SubComponents.HeatTransfer.TwoPhase.ConstantCoefficient(alpha0 = 1000.0))
    annotation (Placement(transformation(extent={{10,74},{-10,94}})));
  .VaporCycle.Experiments.RefrigerationSystem.RefrigerationSystenInit
                                     init(
    redeclare package WorkingFluid = WorkingFluid,
    T_sc=5,
    T_sh=5,
    mflow_start=0.003,
    T_cond_sec=298.15,mflow_evap_sec = 0.05,p_high = 32e5,dp_low = 0.05e5,dp_high = 0.05e5,p_suction = 3.5e5,T_evap_sec = 287.15)
    annotation (Placement(transformation(extent={{-237.95036883497838,162.78118025660086},{-217.95036883497838,182.78118025660086}},rotation = 0.0,origin = {0.0,0.0})));

  .VaporCycle.HeatExchangers.TwoPhaseAir.CrossFlow evaporator(
    redeclare package WorkingFluid = WorkingFluid,
    init_wf(
      m_flow=init.mflow_start,
      p_in=init.p_evap_in,
      dp=init.dp_low,
      h_in=init.h_cond_out,
      dh=init.h_suction - init.h_cond_out),
    redeclare package Air = AirMedium,
    init_air(
      m_flow=init.mflow_evap_sec,
      p_in=1.013e5 + init.dp_evap_fan,
      dp=init.dp_evap_sec,
      T_in=init.T_evap_sec,
      dT=init.dT_evap_sec,
      phi_in=init.phi_evap_sec,
      phi_outlet=init.phi_evap_sec),
    n=5,
    channelDensity=fill(1000, evaporator.n),
    L_air=0.40,
    D_air=0.012,
    L=fill(10/evaporator.n, evaporator.n),
    Dhyd=fill(0.0048, evaporator.n),
    A=fill(.Modelica.Constants.pi*evaporator.Dhyd*evaporator.Dhyd/4, evaporator.n),
    A_heat=fill(evaporator.L/evaporator.n*evaporator.Dhyd*.Modelica.Constants.pi,
        evaporator.n),
    m=fill(1, evaporator.n),
    wallThickness=0.001,
    wallCrossSecArea=evaporator.A_heat,
    wt_start=0.1,
    twoPhaseChannel(use_stabilizer_twoPhaseFraction=true,
        c_pseudo_twoPhaseFraction_const=0.02),hx_type = 2,redeclare replaceable model HeatTransfer = .VaporCycle.Pipes.SubComponents.HeatTransfer.TwoPhase.ConstantCoefficient(alpha0 = 1000.0))
    annotation (Placement(transformation(extent={{-10.0,-41.9510106767273},{10.0,-61.9510106767273}},rotation = 0.0,origin = {0.0,0.0})));

  .VaporCycle.HeatExchangers.TwoPhaseTwoPhase.CapillarySuctionLine SLHX(
    redeclare package PrimaryWorkingFluid = WorkingFluid,
    init_prim(
      m_flow=init.mflow_start,
      p_in=init.p_cond_out - init.dp_pipe,
      p_out=init.p_evap_in,
      h_in=init.h_cond_out,
      h_out=init.h_cond_out,dh = 0),
    init_sec(
      m_flow=init.mflow_start,
      p_in=init.p_suction + init.dp_pipe,
      p_out=init.p_suction,
      h_in=init.h_suction,
      h_out=init.h_suction),
    n_HX=5,
    n_capIn=2,
    n_capOut=2,
    includeMetaStableRegion=false,
    wt_start=0.1,
    use_stabilizer_twoPhaseFraction=true,redeclare replaceable package
      SecondaryWorkingFluid =
        WorkingFluid,
    c_pseudo_twoPhaseFraction_const=0.001,redeclare replaceable model HeatTransfer = .VaporCycle.Pipes.SubComponents.HeatTransfer.TwoPhase.SinglePhaseGnielinski(k_2ph = 3e3),redeclare replaceable model HeatTransfer_sec = .VaporCycle.Pipes.SubComponents.HeatTransfer.TwoPhase.SinglePhaseGnielinski(k_2ph = 3e3),redeclare replaceable record Geometry = .VaporCycle.HeatExchangers.TwoPhaseTwoPhase.SubComponents.GeometryData.CapillaryTubeSuctionLine(heatExchangeLength = 1.0))                                                                   annotation (Placement(transformation(
        extent={{10.0,10.0},{-10.0,-10.0}},
        rotation=90.0,
        origin={-94.0,5.232148639090115})));

  .VaporCycle.Compressors.FixedEfficiencyCompressor compressor(redeclare package
              Medium =
               WorkingFluid, V_MaxDisplacement(displayUnit="ml") = 8e-06,p_start = init.p_suction,h_start = init.h_suction,Eff_isentropic = 0.95)
                                                     annotation (Placement(
        transformation(
        extent={{-10.0,10.0},{10.0,-10.0}},
        rotation=90.0,
        origin={100.0,24.0})));
  inner .VaporCycle.AggregateTwoPhaseProperties aggregateTwoPhaseProperties
    annotation (Placement(transformation(extent={{200,-160},{220,-140}})));
  .VaporCycle.Sensors.MultiDisplaySensor multiDisplaySensor(redeclare package
      Medium = WorkingFluid)
    annotation (Placement(transformation(extent={{-40,94},{-60,74}})));
  .VaporCycle.Sensors.MultiDisplaySensor multiDisplaySensor1(redeclare package
      Medium = WorkingFluid)
    annotation (Placement(transformation(extent={{60,94},{40,74}})));
  .VaporCycle.Sensors.MultiDisplaySensor multiDisplaySensor2(redeclare package
      Medium = WorkingFluid)
    annotation (Placement(transformation(extent={{-60,-36},{-40,-56}})));
  .VaporCycle.Sensors.MultiDisplaySensor multiDisplaySensor3(redeclare package
      Medium = WorkingFluid)
    annotation (Placement(transformation(extent={{40,-36},{60,-56}})));
  .VaporCycle.Sensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot
    annotation (Placement(transformation(extent={{-64,60},{-36,88}})));
  .VaporCycle.Sensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot1
    annotation (Placement(transformation(extent={{36,58},{64,86}})));
  .VaporCycle.Sensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot2
    annotation (Placement(transformation(extent={{-64,-70},{-36,-42}})));
  .VaporCycle.Sensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot3
    annotation (Placement(transformation(extent={{36,-70},{64,-42}})));
  .VaporCycle.Sources.Speed compressor_rotor(speedUnit=.Modelon.ThermoFluid.Choices.SpeedUnit.Hertz)
    annotation (Placement(transformation(extent={{164,20},{156,28}})));
  .Modelica.Blocks.Sources.Constant displacement(k=1)
    annotation (Placement(transformation(extent={{125,-1},{115,9}})));
  .VaporCycle.Fan.IdealFan evaporator_fan(
    redeclare package Medium = AirMedium,
    T_start=init.T_evap_sec,
    p_start=101300,
    mass_flow=init.mflow_evap_sec,
    paraOption=true,
    Tnom=init.T_evap_sec) annotation (Placement(transformation(
        extent={{8,9},{-8,-9}},
        rotation=270,
        origin={20,-86})));
  .VaporCycle.Pipes.AirDuct airDuct(
    redeclare package Medium = AirMedium,
    Across=0.5,
    L=1,
    m={airDuct.Across * airDuct.L * AirMedium.reference_d},
    Rw=0.0240,
    T0={init.T_evap_sec},
    T_start=init.T_evap_sec,
    T_nom=init.T_evap_sec,
    p_start=103500,
    dp_nom=init.dp_case,
    mdot_nom=init.mflow_evap_sec,
    flowResistance(redeclare model Friction =
          .Modelon.ThermoFluid.FlowResistances.FrictionModels.LinearOperatingPointLoss (
           dp0=init.dp_case, m_flow0=init.mflow_evap_sec))) annotation (
      Placement(transformation(
        extent={{-20.427631578947384,-124.83163648346587},{5.572368421052616,-98.83163648346587}},
        rotation=0.0,
        origin={0.0,0.0})));

  .Modelica.Thermal.HeatTransfer.Components.ThermalResistor
    airTemperatureResitance(R=0.078)
    annotation (Placement(transformation(extent={{-34,-150},{-14,-130}})));
  .Modelica.Thermal.HeatTransfer.Components.HeatCapacitor
    thermalCapacitance(C=10 * airDuct.Cp, T(fixed=true, start=init.T_evap_sec))
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={22,-140})));
  .Modelon.ThermoFluid.Sources.Environment_T airTemperature(N=1, T0=306.15)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-56,-140})));
  .VaporCycle.Experiments.RefrigerationSystem.OnOffDefrostController
    onOffDefrostController(
    setpoint=277.15,
    differential=3,
    startTimeOn=100000,
    defrost_sequence=360000,
    fileName=.Modelica.Utilities.Files.loadResource("modelica://VaporCycle/Resources/Data/Testing_Dynamic_Data_comp_cond_evap01.txt"),
    compressorOn=40,
    controllerCompressor_On2(k=true))
    annotation (Placement(transformation(extent={{-202,140},{-178,160}})));

  .Modelon.ThermoFluid.Sources.Environment_T environment_T(N=1, paraOption_T=true)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,120})));
  replaceable .Modelica.Blocks.Sources.Ramp ambientTemperature(
    height=30,
    duration=100,
    offset=init.T_cond_sec,
    startTime=1e6) constrainedby .Modelica.Blocks.Sources.Ramp
    annotation (Placement(transformation(extent={{39,111},{21,129}})));
  replaceable .VaporCycle.Utilities.Visualizers.pH_Diagrams.pH_R134a pHDiagram(
    x=h_Diagram,
    y=.Modelica.Math.log(p_Diagram),
    color={0,0,0},
    rectangle_scale=false) constrainedby
    .Modelon.Visualizers.XYDiagramBackgroundImage annotation (Placement(
        transformation(
        extent={{220.73220151584923,13.422783420173573},{388.7322015158492,181.42278342017357}},
        rotation=0.0,
        origin={0.0,0.0})));
  .VaporCycle.Sensors.AirTemperature airTemperatureSensor(redeclare package
      Medium = AirMedium)
    annotation (Placement(transformation(extent={{-186,-114},{-206,-90}})));
  .VaporCycle.Experiments.RefrigerationSystem.VaporCompressionCycle summary(
    P_compressor=compressor.summary.shaftPower,
    COP=max(summary.P_evaporator / (max(1e-5,summary.P_compressor)),1e-5),
    p_high=condenser.twoPhaseChannel.p[1],
    p_low=compressor.summary.p_suc,
    h_evap_out=evaporator.summary.h_out,
    h_evap_in=evaporator.summary.h_in,
    h_cond_out=condenser.summary.h_out,
    h_cond_in=condenser.summary.h_in,
    m_flow=SLHX.summary.m_flow,
    dp_evap=evaporator.summary.dp,
    dp_cond=condenser.summary.dp,
    T_evap_in_sec=evaporator.summary.T_sec_in,
    T_evap_out_sec=evaporator.summary.T_sec_out,
    T_cond_in_sec=condenser.summary.T_sec_in,
    T_cond_out_sec=condenser.summary.T_sec_out,
    P_condenser=-condenser.summary.Q_flow,
    recLevel=1,
    recQuality=0,
    subcooling=sensorSubCooling.y,
    superheat=sensorEvaporatorSuperHeat.y,
    M=aggregateTwoPhaseProperties.M,
    V=aggregateTwoPhaseProperties.V,
    P_evaporator=-evaporator.summary.Q_flow_sec)
    annotation (Placement(transformation(extent={{-240,120},{-220,140}})));
  .VaporCycle.Sensors.SuperHeatSensor sensorEvaporatorSuperHeat(redeclare package
              Medium = WorkingFluid) annotation (Placement(transformation(
        extent={{-5.50006,-6.50002},{5.49983,6.50002}},
        rotation=0,
        origin={21.5001,-47.5})));
  .VaporCycle.Sensors.SubCoolingSensor sensorSubCooling(redeclare package
      Medium =
        WorkingFluid) annotation (Placement(transformation(
        extent={{-4.00001,-5.00001},{4.00001,5.00001}},
        rotation=180,
        origin={-74,85})));
  .VaporCycle.Sensors.SuperHeatSensor sensorSuctionSuperHeat(redeclare package
      Medium = WorkingFluid) annotation (Placement(transformation(
        extent={{-6.00012,-6.99998},{5.99988,6.99998}},
        rotation=0,
        origin={69,8.0001})));
  .VaporCycle.Tanks.LiquidReceiver
                       receiver(
    redeclare package Medium = WorkingFluid,
    H=0.20,
    D=0.05528,
    H_suction=0.01,
    desiccant=false,
    initFromEnthalpy=true,
    initOpt=.Modelon.ThermoFluid.Choices.InitOptions.initialValues,
    zeta=200,
    staticHead=false,
    p_start=init.p_cond_out - init.dp_pipe,
    h_start=init.h_cond_out,use_L_start = false,L_start = 0.10)
    annotation (Placement(transformation(extent={{-68.0,49.37220591437948},{-88.0,69.37220591437948}},rotation = 0.0,origin = {0.0,0.0})));
  .VaporCycle.Sources.TwoPhaseFlowSource flowSource(
    redeclare package Medium = WorkingFluid,
    use_m_flow_in=true,
    use_Th_in=true,
    p_start=init.p_evap_out)
    annotation (Placement(transformation(extent={{146,-16},{126,-36}})));
  .VaporCycle.Experiments.RefrigerationSystem.ChargeController chargeController(
    ControllerType=2,
    Ti=50,
    yMax=0.01,
    yMin=-0.01,
    duration=20,
    startTime=100000,
    targetCharge=0.385,
    initialCharge=0.385,
    threshold_time=10000)
    annotation (Placement(transformation(extent={{176.0,-55.42683574689795},{156.0,-35.42683574689795}},rotation = 0.0,origin = {0.0,0.0})));
  .Modelica.Blocks.Sources.RealExpression chargeControlActualCharge(y=summary.M)
    annotation (Placement(transformation(
        extent={{10,-9},{-10,9}},
        rotation=0,
        origin={206,-46})));
  .Modelica.Blocks.Sources.RealExpression chargeControlEnthalpy(y=evaporator.portB_prim.h_outflow)
    annotation (Placement(transformation(extent={{216,-86},{196,-66}})));
equation
  connect(multiDisplaySensor.u1, multiDisplayVis_phTmdot.y)
    annotation (Line(points={{-50,84},{-50,74}}, color={0,0,0}));
  connect(multiDisplaySensor2.u1, multiDisplayVis_phTmdot2.y)
    annotation (Line(points={{-50,-46},{-50,-56}}, color={0,0,0}));
  connect(multiDisplaySensor3.u1, multiDisplayVis_phTmdot3.y)
    annotation (Line(points={{50,-46},{50,-56}}, color={0,0,0}));
  connect(multiDisplaySensor1.u1, multiDisplayVis_phTmdot1.y)
    annotation (Line(points={{50,84},{50,72}}, color={0,0,0}));
  connect(SLHX.portB_prim, multiDisplaySensor2.portA) annotation (Line(points={{-100,-4.767851360909884},{-100,-46},{-60,-46}}, color={0,190,0}));
  connect(multiDisplaySensor2.portB, evaporator.portA_prim)
    annotation (Line(points={{-40,-46},{-10,-46},{-10,-45.9510106767273}}, color={0,190,0}));
  connect(multiDisplaySensor3.portB, SLHX.portA_sec) annotation (Line(points={{60,-46},{80,-46},{80,-26},{-88,-26},{-88,-4.767851360909887}}, color={0,190,0}));
  connect(compressor.flange, compressor_rotor.flange)
    annotation (Line(points={{109,23.999999999999996},{109,24},{156,24}}, color={0,0,0}));
  connect(displacement.y, compressor.relativeDisplacement)
    annotation (Line(points={{114.5,4},{106,4},{106,13.999999999999998}}, color={0,0,127}));
  connect(onOffDefrostController.ctrl_compressor, compressor_rotor.inPort)
    annotation (Line(points={{-177,150},{188,150},{188,24},{164.8,24}}, color={0,
          0,127}));
  connect(ambientTemperature.y, environment_T.T_in)
    annotation (Line(points={{20.1,120},{10,120}}, color={0,0,127}));
  connect(environment_T.port[1], condenser.ambient) annotation (Line(points={{-1.11022e-15,
          110},{-1.11022e-15,102},{0,102},{0,94}}, color={191,0,0}));
  connect(condenser.portA_prim, multiDisplaySensor1.portB)
    annotation (Line(points={{10,84},{40,84}}, color={0,190,0}));
  connect(condenser.portB_prim, multiDisplaySensor.portA)
    annotation (Line(points={{-10,84},{-40,84}}, color={0,190,0}));
  connect(evaporator.portB_prim, sensorEvaporatorSuperHeat.portA) annotation (
      Line(points={{10,-45.9510106767273},{10,-46.4167},{16,-46.4167}}, color={0,190,
          0}));
  connect(sensorEvaporatorSuperHeat.portB, multiDisplaySensor3.portA)
    annotation (Line(points={{26.9999,-46.4167},{33.5,-46.4167},{33.5,-46},{40,-46}},
        color={0,190,0}));
  connect(sensorSuctionSuperHeat.portB, compressor.portA) annotation (Line(
        points={{74.9999,9.16676},{100,9.16676},{100,14}}, color={0,190,0}));
  connect(multiDisplaySensor.portB, sensorSubCooling.portA) annotation (Line(
        points={{-60,84},{-65,84},{-65,84.1667},{-70,84.1667}}, color={0,190,0}));
  connect(multiDisplaySensor1.portA, compressor.portB)
    annotation (Line(points={{60,84},{100,84},{100,34}}, color={0,190,0}));
  connect(SLHX.portB_sec, sensorSuctionSuperHeat.portA) annotation (Line(points={{-88,15.232148639090113},{-88,26},{62.9999,26},{62.9999,9.16676}},
                                                             color={0,190,0}));
  connect(sensorSubCooling.portB, receiver.portA) annotation (Line(points={{-78,84.1667},{-78,76},{-74,76},{-74,69.37220591437948}},
                                                color={0,190,0}));
  connect(chargeControlEnthalpy.y,flowSource. h_in) annotation (Line(points={{195,-76},
          {137.2,-76},{137.2,-33}},      color={0,0,127}));
  connect(flowSource.m_flow_in,chargeController. m_flow) annotation (Line(
        points={{141.2,-31},{141.2,-45.42683574689795},{155,-45.42683574689795}}, color={0,0,127}));
  connect(flowSource.port, SLHX.portA_sec) annotation (Line(points={{128.2,-26},{-88,-26},{-88,-4.767851360909887}}, color={0,190,0}));
  connect(chargeController.charge,chargeControlActualCharge. y) annotation (
      Line(points={{177,-44.82683574689795},{187,-44.82683574689795},{187,-46},{195,-46}}, color={0,0,127}));
  connect(onOffDefrostController.T_evap_in, airTemperatureSensor.outPort)
    annotation (Line(points={{-203,150},{-208,150},{-208,-84},{-196,-84},{-196,-90}},
        color={0,0,127}));
    connect(receiver.portB,SLHX.portA_prim) annotation(Line(points = {{-82,69.37220591437948},{-82,74},{-100,74},{-100,15.232148639090116}},color = {0,190,0}));
    connect(evaporator.portB_sec,airDuct.a) annotation(Line(points={{-10,-57.9510106767273},{-26,-57.9510106767273},{-26,-111.83163648346587},{-20.427631578947384,-111.83163648346587}},  color = {85,170,255}));
    connect(airTemperatureSensor.port,airDuct.a) annotation(Line(points={{-186,-100},{-26,-100},{-26,-111.83163648346587},{-20.427631578947384,-111.83163648346587}},  color = {85,170,255}));
    connect(airDuct.q,thermalCapacitance.port) annotation(Line(points={{-7.427631578947384,-120.93163648346587},{-7.427631578947384,-140},{12,-140}},  color = {191,0,0}));
    connect(airTemperatureResitance.port_b,thermalCapacitance.port) annotation(Line(points = {{-14,-140},{12,-140}},color = {191,0,0}));
    connect(airTemperatureResitance.port_a,airTemperature.port[1]) annotation(Line(points = {{-34,-140},{-46,-140}},color = {191,0,0}));
    connect(evaporator_fan.portB,evaporator.portA_sec) annotation(Line(points = {{20,-78},{20,-57.9510106767273},{10,-57.9510106767273}},color = {85,170,255}));
    connect(onOffDefrostController.ctrl_evapFlow,evaporator_fan.massFlow) annotation(Line(points = {{-177,144},{-160,144},{-160,-86},{11,-86}},color = {0,0,127}));
    connect(airDuct.b,evaporator_fan.portA) annotation(Line(points={{5.572368421052616,-111.83163648346587},{20,-111.83163648346587},{20,-94}},  color = {85,170,255}));
  annotation (
    Diagram(coordinateSystem(extent={{-240,-180},{460,180}})),
    experiment(
      StopTime=16000,
      Tolerance=1e-07,
      __Dymola_Algorithm="Cvode"),Documentation(info = "<html><p>This is a template for a refrigeration system  with R-134 as default refrigerant. A vapor compression cycle with a&nbsp;natural convection wire and tube condenser (<a href=\"modelica://VaporCycle.HeatExchangers.TwoPhaseAir.WireAndTubeHX\">WireAndTubeHX</a>) , a suction line heat exchanger (<a href=\"modelica://VaporCycle.HeatExchangers.TwoPhaseTwoPhase.CapillarySuctionLine\">CapillarySuctionLine</a> ), a generic cross flow air evaporator (<a href=\"modelica://VaporCycle.HeatExchangers.TwoPhaseAir.CrossFlow\">CrossFlow</a>), and a fixed displacement fixed efficiency&nbsp;compressor (<a href=\"modelica://VaporCycle.Compressors.FixedEfficiencyCompressor\">FixedEfficiencyCompressor</a>) as default components.&nbsp; &nbsp;</p><p>The charge of the system is regulated by the receiver and a charge controller (<a href=\"modelica://VaporCycle.Experiments.RefrigerationSystem.ChargeController\">ChargeController</a>).&nbsp; The compressor, fan, and evaporator&nbsp; are controlled by the&nbsp; on-off defrost controller (<a href=\"modelica://VaporCycle.Experiments.RefrigerationSystem.OnOffDefrostController\">OnOffDefrostController</a>).&nbsp;&nbsp;</p><p>The system uses air as secondary fluid at the evaporator side, and a prescribed temperature as boundary condition at the condenser side.</p><p>The refrigeration cabinet is modelled by a simplified thermal capacitance, with a thermal resistance for heat losses.&nbsp;</p><p><br></p><p><br></p><p><a href=\"modelica://VaporCycle.HeatExchangers.TwoPhaseAir.WireAndTubeHX\"></a></p></html>",
        revisions="<html>
Copyright &copy; 2004-2023, MODELON AB <br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br />This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br />or by other means have their origin from any Modelon Library.
</html>"));
end RefrigerationSystemWithSLHX;
