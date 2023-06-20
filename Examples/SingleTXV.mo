within Examples;

model SingleTXV
  "AC - HP model switch, single expansion valve with different superheat measurement per mode"

 parameter Real startHeatCycle=150   "start Time for changing from Refrigeration to Heat Pump cycle";
 parameter Real timeDelay=1     "time delay for feed forward after switching to HP mode";

  .AirConditioning.Examples.HeatPumpMode.Components.CycleSummary summary(
    final P_hex_indoor=hex_Indoor.summary.P_evaporator,
    final P_compressor=compressor.summary.P_shaft,
    final P_hex_outdoor=hex_Outdoor.summary.Qdot_refTotal,
    final COP=if acModeTable.y then hex_Indoor.summary.Qdot_refTotal/max(1e-5,
        summary.P_compressor) else -hex_Indoor.summary.Qdot_refTotal/max(1e-5,
        summary.P_compressor),
    final p_high=compressor.summary.p_discharge,
    final p_low=compressor.summary.p_suction,
    final Superheat=expansionValve.DeltaT_SH,
    final Subcooling=if acModeTable.y then subCoolingSensorOutdoor.TempSC else
        subCoolingSensorIndoor.TempSC,
    final Tair_hex_indoor_in=hex_Indoor.summary.Tair_in,
    final Tair_hex_indoor_out=hex_Indoor.summary.Tair_out,
    final Tair_hex_outdoor_in=hex_Outdoor.summary.Tair_in,
    final Tair_hex_outdoor_out=hex_Outdoor.summary.Tair_out,
    final h_hex_indoor_out=hex_Indoor.summary.h_out,
    final h_hex_indoor_in=hex_Indoor.summary.h_in,
    final h_hex_outdoor_out=hex_Outdoor.summary.h_out,
    final h_hex_outdoor_in=hex_Outdoor.summary.h_in,
    final mdot=expansionValve.summary.mdot,
    final dp_hex_indoor=hex_Indoor.summary.dp_ref,
    final dp_hex_outdoor=hex_Outdoor.summary.dp_ref,
    final recLevel=receiver.summary.relLevel,
    final recQuality=receiver.summary.quality,
    final M_ref=aggregateTwoPhaseProperties.M,
    final V_ref=aggregateTwoPhaseProperties.V);

extends .AirConditioning.SubComponents.Records.TemplateData.TestbenchPars(
      redeclare package Medium =
        .AirConditioning.ThermoFluidPro.Media.TableBasedProperties.R134a);
        extends .Modelon.Icons.Experiment;
inner .AirConditioning.AggregateTwoPhaseProperties aggregateTwoPhaseProperties annotation(Placement(transformation(extent = {{76.0,32.0},{96.0,52.0}},origin = {0.0,0.0},rotation = 0.0)));
protected
  .Modelica.Units.SI.Pressure[5] p_Diagram=if acModeTable.y then {compressor.summary.p_suction,compressor.summary.p_discharge,
      expansionValve.summary.p_in,expansionValve.summary.p_out,compressor.summary.p_suction} else {compressor.summary.p_suction,
      compressor.summary.p_discharge,expansionValve.summary.p_out,expansionValve.summary.p_in,compressor.summary.p_suction}
    "pressures for p-h diagram";
  .Modelica.Units.SI.SpecificEnthalpy[5] h_Diagram=if acModeTable.y then {compressor.summary.h_suction,compressor.summary.h_discharge,
      expansionValve.valve.h,hex_Indoor.summary.h_in,compressor.summary.h_suction} else {compressor.summary.h_suction,
      compressor.summary.h_discharge,hex_Indoor.summary.h_in,expansionValve.valve.h,compressor.summary.h_suction}
    "specific enthalpies for p-h diagram";
  parameter .Modelica.Units.SI.Time startTime(fixed=false);

public
.AirConditioning.SubComponents.Records.InitData.CycleInit init(
    redeclare replaceable package Medium = Medium,
    dp_high(displayUnit="bar") = 5000,
    dp_low(displayUnit="bar") = 70000,
    charge=summary.SpecificCharge,
    V_ref=summary.V_ref,
    mdot_init=0.03,
    compSpeed=22,
    mdotCond=0.5,
    mdotEvap=0.095,
    steadyState=false,
    steadyPressure=false,
    p_receiver=receiver.summary.p,
    M_receiver=receiver.summary.M_ref,
    V_receiver=receiver.summary.V_ref,
    charge_init=150,
    T_sh=5,
    T_sc=20,
    p_high=2050000,
    p_suction=400000,
    TairCond=315,
    TairEvap=304.15) annotation (Placement(transformation(extent={{128,32},{160,
            52}}, rotation=0)));

  .AirConditioning.PipesAndVolumes.PipeAdiabatic pipe1(
    n=1,
    redeclare replaceable package Medium = Medium,
    geo(D=0.02, L=0.5),
    init(
      steadyState=init.steadyState,
      steadyPressure=init.steadyPressure,
      mdot0=init.mdot_init,
      h_in=init.h_cond_in,
      h_out=init.h_cond_in,
      p_in=init.p_high,
      p_out=init.p_high - init.dp_pipe + init.dp_pipe/10),
    redeclare replaceable model FrictionLoss =
        .AirConditioning.ThermoFluidPro.PressureLoss.PressureLossTD (d0=fill(100.0, pipe1.n),
          mdot0=0.03)) annotation (Placement(transformation(
        origin={40,20},
        extent={{-10,-10},{10,10}},
        rotation=90)));
  .AirConditioning.PipesAndVolumes.PipeAdiabatic pipe3(
    n=1,
    redeclare replaceable package Medium = Medium,
    geo(D=0.008, L=0.3),
    init(
      steadyState=init.steadyState,
      steadyPressure=init.steadyPressure,
      mdot0=init.mdot_init,
      h_in=init.h_cond_out,
      h_out=init.h_cond_out,
      p_in=init.p_high - init.dp_high - init.dp_pipe - init.dp_pipe/10,
      p_out=init.p_high - init.dp_high - 2*init.dp_pipe),
    redeclare replaceable model FrictionLoss =
        .AirConditioning.ThermoFluidPro.PressureLoss.PressureLossTD (d0=fill(1200.0, pipe3.n),
          mdot0=0.03)) annotation (Placement(transformation(
        origin={-136,10},
        extent={{10,-10},{-10,10}},
        rotation=90)));
  .AirConditioning.PipesAndVolumes.PipeAdiabatic pipe4(
    n=1,
    redeclare replaceable package Medium = Medium,
    geo(D=0.02, L=0.5),
    init(
      steadyState=init.steadyState,
      steadyPressure=init.steadyPressure,
      mdot0=init.mdot_init,
      h_in=init.h_suction,
      h_out=init.h_suction,
      p_in=init.p_suction + init.dp_pipe,
      p_out=init.p_suction + init.dp_pipe/10),
    redeclare replaceable model FrictionLoss =
        .AirConditioning.ThermoFluidPro.PressureLoss.PressureLossTD (
        d0=fill(12.0, pipe4.n),
        dp0=5000,
        mdot0=0.03),
    quasiStatic=false) annotation (Placement(transformation(
        origin={40,-62},
        extent={{-10,-10},{10,10}},
        rotation=90)));
.AirConditioning.ControllersAndSensors.SuperHeatSensor superHeat(redeclare
      replaceable package Medium =
                     Medium)
  annotation (Placement(transformation(extent={{4,-74},{24,-94}}, rotation=0)));
  replaceable .AirConditioning.HeatExchangers.Evaporator hex_Outdoor(
    n_segAir=1,
    redeclare replaceable package Medium = Medium,
    redeclare model RefrigerantFrictionLossModel =
        .AirConditioning.ThermoFluidPro.PressureLoss.PLossHexChannel.PlossCommon,
    final HX_Init=init.condInit,
    redeclare model HTCoefficientRefSide =
        .AirConditioning.ThermoFluidPro.HeatTransfer.HTTwoPhaseMedium.KcOverallR134a,
    n_segRef=2,
    refrigerant(
      use_stabilizer_twoPhaseFraction=true,
      c_pseudo_twoPhaseFraction_const=0.01,
      twoPhaseFraction_start=ones(hex_Outdoor.refrigerant.n)))
                annotation (choicesAllMatching, Placement(transformation(extent=
           {{4,44},{-36,84}}, rotation=0)));

  replaceable .AirConditioning.HeatExchangers.Evaporator hex_Indoor(
    n_segAir=1,
    redeclare replaceable package Medium = Medium,
    redeclare model HTCoefficientRefSide =
        .AirConditioning.ThermoFluidPro.HeatTransfer.HTTwoPhaseMedium.KcOverallR134a,
    redeclare model RefrigerantFrictionLossModel =
        .AirConditioning.ThermoFluidPro.PressureLoss.PLossHexChannel.PlossCommon,
    final HX_Init=init.evapInit,
    refrigerant(use_stabilizer_twoPhaseFraction=true,
        c_pseudo_twoPhaseFraction_const=0.01))
                                 annotation (choicesAllMatching, Placement(
        transformation(extent={{-70,-90},{-30,-50}}, rotation=0)));

  replaceable .AirConditioning.Compressors.GenericExternalControl compressor(
    redeclare replaceable package Medium = Medium,
    MaximumDisplacement=16e-5,
    init(
      steadyState=false,
      steadyPressure=false,
      final p0=init.p_suction,
      final h0=init.h_suction)) constrainedby
    .AirConditioning.Compressors.Templates.VariableDisplacement(
    redeclare replaceable package Medium = Medium,
    init(
      steadyState=false,
      steadyPressure=false,
      p0=init.p_suction,
      h0=init.h_suction),
    compressor(CompressorSpeed(start=init.compSpeed)))
    "simple parameterization for R134a compressor" annotation (Placement(
        transformation(
        origin={96,-6},
        extent={{-10,10},{10,-10}},
        rotation=90)));
.AirConditioning.ThermoFluidPro.Components.Compressors.Speed rps(
  phi(fixed=true, start=0.0),
  exact=true) "rotational speed of compressor"
  annotation (Placement(transformation(extent={{132,-16},{120,0}},
                                                                 rotation=0)));
replaceable .Modelica.Blocks.Sources.Constant RelVol(k=1.0)
  constrainedby .Modelica.Blocks.Interfaces.SO "relative displacement volume"
                                 annotation (Placement(transformation(extent={{140,-38},
            {128,-26}},          rotation=0)));
  .AirConditioning.Valves.SimpleTXV                 expansionValve(
    steadySuperheat=false,
    enableFeedForward=true,
    redeclare replaceable package Medium = Medium,
    yMax=0.2,
    mdot0=init.mdot_init,
    geo(D=0.0075, L=0.1),
    yInit=0.03,
    k=-0.002,
    SuperHeatSetPoint=init.T_sh,
    Ti=17,
    yMin=0.002,
    init(
      h0=init.h_cond_out,
      steadyState=true,
      steadyPressure=true,
      p0=init.p_high - init.dp_high - 2*init.dp_pipe - init.dp_pipe/10))
    annotation (Placement(transformation(
        origin={-136,-40},
        extent={{-10,10},{10,-10}},
        rotation=90)));
  .AirConditioning.Reservoirs.Air.FlowSourceAir airIn_evap annotation (Placement(
        transformation(
        origin={-15,-59},
        extent={{-9,9},{9,-9}},
        rotation=180)));
  .AirConditioning.Reservoirs.Air.FlowSourceAir airIn_cond annotation (Placement(
        transformation(extent={{-69,67},{-51,85}}, rotation=0)));
  .AirConditioning.Reservoirs.Air.FlowSinkAir airOut_cond annotation (Placement(
        transformation(
        origin={20,76},
        extent={{8,8},{-8,-8}},
        rotation=0)));
  .AirConditioning.Reservoirs.Air.FlowSinkAir airOut_evap annotation (Placement(
        transformation(
        origin={-106,-58},
        extent={{8,-8},{-8,8}},
        rotation=180)));
  replaceable .Modelica.Blocks.Sources.Ramp Cond_tair(
    duration=1,
    height=-26.85,
    final offset=init.TairCond,
    startTime=startHeatCycle) constrainedby .Modelica.Blocks.Interfaces.SO
    annotation (Placement(transformation(extent={{-32,94},{-46,108}}, rotation=0)));
  replaceable .Modelica.Blocks.Sources.Ramp Evap_tair(
    height=-11,
    final offset=init.TairEvap,
    duration=1,
    startTime=startHeatCycle) constrainedby .Modelica.Blocks.Interfaces.SO
    annotation (Placement(transformation(
        origin={-36,-32},
        extent={{7,7},{-7,-7}},
        rotation=180)));
  replaceable .Modelica.Blocks.Sources.Ramp Evap_mair(
    height=init.mdotCond - init.mdotEvap,
    final offset=init.mdotEvap,
    duration=2,
    startTime=startHeatCycle) constrainedby .Modelica.Blocks.Interfaces.SO
    annotation (Placement(transformation(extent={{19,-39},{5,-25}}, rotation=0)));
.AirConditioning.Receivers.WaterAccumulator waterAccumulator
  annotation (Placement(transformation(extent={{-58,-106},{-44,-92}},
          rotation=0)));
.AirConditioning.Visualizers.RealValue valueTime(precision=1, number=time)
  annotation (Placement(transformation(extent={{118,-86},{144,-66}},
                                                                   rotation=0)));
.AirConditioning.Visualizers.RealValue valueP_evap(precision=0, number=summary.P_hex_indoor)
                    annotation (Placement(transformation(extent={{118,-106},{144,
            -86}}, rotation=0)));
.AirConditioning.Visualizers.RealValue valueCOP(                   precision=1,
      number=summary.COP)
  annotation (Placement(transformation(extent={{118,-126},{144,-106}},
                                                                    rotation=
            0)));
.AirConditioning.Visualizers.RealValue valueCondensate(precision=1, number=
       waterAccumulator.M*1000)
                               annotation (Placement(transformation(extent={{-88,
            -110},{-62,-90}},     rotation=0)));
replaceable .AirConditioning.Visualizers.PH_R134a_logp                      PHDiagram(
x=h_Diagram,
y=.Modelica.Math.log(p_Diagram),
color={255,0,0}) constrainedby .Modelon.Visualizers.XYDiagramBackgroundImage(
x=h_Diagram,
y=.Modelica.Math.log(p_Diagram),
color={255,0,0}) annotation (choicesAllMatching, Placement(transformation(
          extent={{100,60},{160,120}},rotation=0)));

protected
  .AirConditioning.Visualizers.FourValueLegend legend
                                   annotation (Placement(transformation(
          extent={{-100,-22},{-68,2}},rotation=0)));

public
  inner .AirConditioning.System_ACL system_ACL(positiveFlow_ref=false)
    annotation (Placement(transformation(extent={{100,32},{120,52}})));
  .AirConditioning.ControllersAndSensors.MultiDisplaySensor multiDisplaySensor(
      redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{15,55},{5,45}})));
  .AirConditioning.ControllersAndSensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot
    annotation (Placement(transformation(extent={{27,13},{-7,47}})));
  .AirConditioning.ControllersAndSensors.MultiDisplaySensor multiDisplaySensor1(
      redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-62,58},{-78,42}})));
  .AirConditioning.ControllersAndSensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot1
    annotation (Placement(transformation(extent={{-52,12},{-88,48}})));
  .AirConditioning.ControllersAndSensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot2
    annotation (Placement(transformation(extent={{0,-114},{-32,-82}})));
  .AirConditioning.ControllersAndSensors.MultiDisplaySensor multiDisplaySensor2(
      redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-24,-76},{-8,-92}})));
  .AirConditioning.Valves.FourPortsValve fourPortsValve(
    redeclare package Medium = Medium,
    Kv0=2,
    geoA(
      L=0.1,
      A=7.8e-5,
      C=0.1),
    geoD(
      L=0.1,
      A=7.8e-5,
      C=0.1),
    initD(
        steadyState=true,
        h0=init.h_cond_in,
        p0=init.p_high),
    initA(
      steadyState=true,
      h0=init.h_suction,
      p0=init.p_suction + 0.05*init.dp_pipe),
    enable_visualization=true)                annotation (Placement(
        transformation(
        extent={{11,-20},{-11,20}},
        rotation=180,
        origin={60,-20})));
.AirConditioning.ControllersAndSensors.SuperHeatSensor superHeat1(redeclare
      replaceable package Medium = Medium)
  annotation (Placement(transformation(extent={{40,40},{20,60}},  rotation=0)));
  .AirConditioning.ControllersAndSensors.MultiDisplaySensor multiDisplaySensor3(
      redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-122,-76},{-106,-92}})));
  .AirConditioning.ControllersAndSensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot3
    annotation (Placement(transformation(extent={{-98,-110},{-130,-78}})));
.AirConditioning.Receivers.WaterAccumulator waterAccumulator1
  annotation (Placement(transformation(extent={{-46,24},{-32,38}},
          rotation=0)));
.AirConditioning.Visualizers.RealValue valueCondensate1(precision=1, number=
        waterAccumulator1.M*1000)
                               annotation (Placement(transformation(extent={{-50,4},
            {-24,24}},            rotation=0)));
  replaceable .Modelica.Blocks.Sources.Ramp Cond_phi(
    duration=1,
    height=init.phiEvap - init.phiCond,
    final offset=init.phiCond,
    startTime=startHeatCycle) constrainedby .Modelica.Blocks.Sources.Ramp
    annotation (Placement(transformation(extent={{-97,67},{-83,81}}, rotation=0)));
  replaceable .Modelica.Blocks.Sources.Ramp Evap_phi(
    height=init.phiCond - init.phiEvap,
    final offset=init.phiEvap,
    duration=1,
    startTime=startHeatCycle) constrainedby .Modelica.Blocks.Sources.Ramp
    annotation (Placement(transformation(
        origin={12,-54},
        extent={{-7,7},{7,-7}},
        rotation=180)));
  .AirConditioning.ControllersAndSensors.AirTemperature airTemperature(isCelsius=
       true)
    annotation (Placement(transformation(extent={{-74,-70},{-94,-48}})));
  replaceable .Modelica.Blocks.Sources.Ramp Cond_mair(
    height=0,
    final offset=init.mdotCond,
    duration=2,
    startTime=startHeatCycle) constrainedby .Modelica.Blocks.Sources.Ramp
    annotation (Placement(transformation(extent={{-97,95},{-83,109}},  rotation=
           0)));
  .AirConditioning.Receivers.SuctionSideAccumulator receiver(
    redeclare replaceable package Medium = Medium,
    Di=0.053,
    H=0.2,
    H_des=0.05,
    zeta=2000,
    desiccant=true,
    H_OutMix=0.05,
    init(
      steadyState=false,
      steadyPressure=init.steadyPressure,
      h0=init.h_suction,
      p0=init.p_suction + 0.03*init.dp_pipe))
    annotation (Placement(transformation(extent={{74,-62},{94,-42}}, rotation=
           0)));
  .AirConditioning.ControllersAndSensors.SubCoolingSensor
    subCoolingSensorOutdoor(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-102,40},{-122,60}})));
  .AirConditioning.ControllersAndSensors.SubCoolingSensor subCoolingSensorIndoor(
      redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-96,-94},{-76,-74}})));
  .Modelica.Blocks.Sources.BooleanTable acModeTable(table={startHeatCycle},
      startValue=true) "True for A/C mode, false for heat pump mode"
    annotation (Placement(transformation(extent={{-180,100},{-160,120}})));
  .Modelica.Blocks.Logical.Switch switch1 annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-160,70})));
  .Modelica.Blocks.Continuous.FirstOrder firstOrder(
    k=1,
    T=1,
    initType=.Modelica.Blocks.Types.Init.SteadyState) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-160,38})));
  .Modelica.Blocks.Continuous.FirstOrder firstOrder1(
    k=1,
    T=0.5,
    initType=.Modelica.Blocks.Types.Init.SteadyState) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={60,90})));
  .Modelica.Blocks.Logical.Switch switch2 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={30,108})));
  .Modelica.Blocks.Sources.Constant const(k=1)
    annotation (Placement(transformation(extent={{-4,114},{6,124}})));
  .Modelica.Blocks.Sources.Constant const1(k=0)
    annotation (Placement(transformation(extent={{-4,92},{6,102}})));
  .Modelica.Blocks.Sources.Step     FF(height=0.04,  startTime=startHeatCycle +
        timeDelay)
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-160,-100})));
  .Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising=0.005)
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-160,-70})));
  replaceable .Modelica.Blocks.Sources.Trapezoid
                                           Speed(
    amplitude=-init.compSpeed + 3,
    rising=0.1,
    width=0.2,
    falling=0.1,
    period=1e6,
    offset=init.compSpeed,
    startTime=startHeatCycle)
                constrainedby .Modelica.Blocks.Sources.Trapezoid
                                                            annotation (
      Placement(transformation(extent={{156,-16},{144,-4}}, rotation=0)));
initial equation
  startTime = time;
equation
  assert(Medium.mediumName == init.mediumName, "Inconsistent medium definitions in model: init.Medium=" + init.mediumName +", and Medium = "+Medium.mediumName);
  if receiver.Separator.stateChoice == .AirConditioning.ThermoFluidPro.Types.ThermoStates.UM_States then
    when sample(startTime,.Modelica.Constants.inf) and init.initType==1 then
      reinit(receiver.Separator.M[1],init.M_corrected);
      reinit(receiver.Separator.U[1],init.U_corrected);
    end when;
  else
    when sample(startTime,.Modelica.Constants.inf) and init.initType==1 then
      reinit(receiver.Separator.h[1],init.h_receiver);
    end when;
  end if;

connect(rps.flange,  compressor. Flange_a1)
  annotation (Line(points={{120,-8},{120,-6},{105,-6}},          color={0,0,0},
      thickness=0.5));
connect(RelVol.y,      compressor. relative_Vol) annotation (Line(
      points={{127.4,-32},{103.5,-32},{103.5,-15.5}},
      color={0,0,255},
      thickness=0.5));
connect(superHeat.b, pipe4.a)       annotation (Line(
      points={{24,-84},{40,-84},{40,-72}},
      color={0,191,0},
      thickness=0.5));
connect(pipe3.b, expansionValve.a)
  annotation (Line(
      points={{-136,0},{-136,-30}},
      color={0,191,0},
      thickness=0.5));
  connect(hex_Indoor.water_out, waterAccumulator.w_in) annotation (Line(
      points={{-40,-87.6},{-40,-96.2},{-45.4,-96.2}},
      color={0,0,255},
      thickness=0.5));
  connect(airIn_cond.b,hex_Outdoor. air_in) annotation (Line(
      points={{-51,76},{-36,76}},
      color={0,127,255},
      thickness=0.5));
connect(airIn_cond.T_in,Cond_tair.y)        annotation (Line(
      points={{-57.3,84.55},{-56,84.55},{-56,101},{-46.7,101}},
      color={0,0,255},
      thickness=0.5));
  connect(airIn_evap.b, hex_Indoor.air_in) annotation (Line(
      points={{-24,-59},{-30,-58}},
      color={0,127,255},
      thickness=0.5));
connect(Evap_tair.y,       airIn_evap.T_in) annotation (Line(
      points={{-28.3,-32},{-18,-32},{-18,-46},{-17.7,-46},{-17.7,-50.45}},
      color={0,0,255},
      thickness=0.5));
  connect(airOut_cond.a,hex_Outdoor. air_out) annotation (Line(
      points={{12,76},{4,76}},
      color={0,127,255},
      thickness=0.5));
connect(Evap_mair.y,       airIn_evap.M_flow_in) annotation (Line(
      points={{4.3,-32},{-11.4,-32},{-11.4,-50.45}},
      color={0,0,255},
      thickness=0.5));
  connect(multiDisplayVis_phTmdot.y, multiDisplaySensor.u)
    annotation (Line(points={{10,30},{10,50}},                 color={0,0,0}));
  connect(multiDisplayVis_phTmdot1.y, multiDisplaySensor1.u)
    annotation (Line(points={{-70,30},{-70,50}}, color={0,0,0},
      thickness=0.5));
  connect(multiDisplaySensor1.portA,hex_Outdoor. b1) annotation (Line(
      points={{-62,50},{-48,50},{-48,49.6},{-36,49.6}},
      color={0,191,0},
      thickness=0.5));
  connect(multiDisplayVis_phTmdot2.y, multiDisplaySensor2.u)
    annotation (Line(points={{-16,-98},{-16,-84}},         color={0,0,0},
      thickness=0.5));
  connect(hex_Indoor.b1, multiDisplaySensor2.portA) annotation (Line(
      points={{-30,-84.4},{-29,-84.4},{-29,-84},{-24,-84}},
      color={0,191,0},
      thickness=0.5));
  connect(multiDisplaySensor2.portB, superHeat.a) annotation (Line(points={{-8,-84},
          {4,-84}},                        color={0,191,0},
      thickness=0.5));
  connect(hex_Outdoor.a1, multiDisplaySensor.portB) annotation (Line(
      points={{4,49.6},{2,49.6},{2,50},{5,50}},
      color={0,191,0},
      thickness=0.5));
  connect(pipe4.b,fourPortsValve. a) annotation (Line(points={{40,-52},{40,-26},
          {49,-26}},           color={0,191,0}));
  connect(pipe1.a,fourPortsValve. c)
    annotation (Line(points={{40,10},{40,-16},{46,-16},{46,-14},{49,-14}},
                                                         color={0,191,0}));
  connect(compressor.DischargePort,fourPortsValve. d) annotation (Line(points={{96,4},{
          96,10},{76,10},{76,-14},{71,-14}},         color={0,191,0}));
  connect(pipe1.b, superHeat1.a)
    annotation (Line(points={{40,30},{40,50}},         color={0,191,0}));
  connect(superHeat1.b, multiDisplaySensor.portA)
    annotation (Line(points={{20,50},{15,50}}, color={0,191,0}));
  connect(multiDisplayVis_phTmdot3.y, multiDisplaySensor3.u)
    annotation (Line(points={{-114,-94},{-114,-84}}, color={0,0,0}));
  connect(expansionValve.b, multiDisplaySensor3.portA) annotation (Line(points={{-136,
          -50},{-136,-84},{-122,-84}},        color={0,191,0}));
  connect(waterAccumulator1.w_in,hex_Outdoor. water_out) annotation (Line(
      points={{-33.4,33.8},{-33.4,38.9},{-26,38.9},{-26,46.4}},
      color={0,0,255},
      thickness=0.5));
  connect(Cond_phi.y, airIn_cond.phi_in) annotation (Line(points={{-82.3,74},{
          -76,74},{-76,80.5},{-68.55,80.5}},
                                         color={0,0,127}));
  connect(Evap_phi.y, airIn_evap.phi_in) annotation (Line(points={{4.3,-54},{-6.45,
          -54},{-6.45,-54.5}},              color={0,0,127}));
  connect(hex_Indoor.air_out, airTemperature.a) annotation (Line(points={{-70,-58},
          {-74,-58}},           color={85,170,255}));
  connect(airTemperature.b, airOut_evap.a) annotation (Line(points={{-94,-58},{-98,
          -58}},                 color={85,170,255}));
  connect(Cond_mair.y, airIn_cond.M_flow_in) annotation (Line(points={{-82.3,102},
          {-63.6,102},{-63.6,84.55}},      color={0,0,127}));
  connect(fourPortsValve.b, receiver.FlowIn)
    annotation (Line(points={{71,-26},{75.4,-26},{75.4,-45}},
                                                            color={0,191,0}));
  connect(receiver.FlowOut, compressor.SuctionPort)
    annotation (Line(points={{93,-45},{96,-45},{96,-16}}, color={0,191,0}));
  connect(multiDisplaySensor3.portB, subCoolingSensorIndoor.a)
    annotation (Line(points={{-106,-84},{-96,-84}}, color={0,191,0}));
  connect(subCoolingSensorIndoor.b, hex_Indoor.a1)
    annotation (Line(points={{-76,-84},{-70,-84.4}}, color={0,191,0}));
  connect(multiDisplaySensor1.portB, subCoolingSensorOutdoor.a)
    annotation (Line(points={{-78,50},{-102,50}}, color={0,191,0}));
  connect(subCoolingSensorOutdoor.b, pipe3.a)
    annotation (Line(points={{-122,50},{-136,50},{-136,20}}, color={0,191,0}));
  connect(superHeat.outPort, switch1.u1) annotation (Line(points={{14,-94},{14,-108},
          {-178,-108},{-178,82},{-168,82}}, color={0,0,127}));
  connect(acModeTable.y, switch1.u2) annotation (Line(points={{-159,110},{-160,110},
          {-160,82}}, color={255,0,255}));
  connect(superHeat1.outPort, switch1.u3) annotation (Line(points={{30,60},{30,88},
          {-152,88},{-152,82}}, color={0,0,127}));
  connect(switch1.y, firstOrder.u)
    annotation (Line(points={{-160,59},{-160,50}}, color={0,0,127}));
  connect(firstOrder.y, expansionValve.DeltaT_SH) annotation (Line(points={{-160,27},
          {-160,-46},{-145.6,-46}},         color={0,0,127}));
  connect(firstOrder1.y, fourPortsValve.u_main)
    annotation (Line(points={{60,79},{60,-2.00004}},     color={0,0,127}));
  connect(switch2.y, firstOrder1.u)
    annotation (Line(points={{41,108},{60,108},{60,102}}, color={0,0,127}));
  connect(const.y, switch2.u1)
    annotation (Line(points={{6.5,119},{18,119},{18,116}}, color={0,0,127}));
  connect(const1.y, switch2.u3)
    annotation (Line(points={{6.5,97},{18,97},{18,100}}, color={0,0,127}));
  connect(acModeTable.y, switch2.u2) annotation (Line(points={{-159,110},{-160,110},
          {-160,114},{-10,114},{-10,108},{18,108}}, color={255,0,255}));
  connect(FF.y, slewRateLimiter.u)
    annotation (Line(points={{-160,-93.4},{-160,-77.2}}, color={0,0,127}));
  connect(slewRateLimiter.y, expansionValve.FF) annotation (Line(points={{-160,
          -63.4},{-160,-42},{-145.6,-42}},
                                  color={0,0,127}));
  connect(Speed.y, rps.inPort) annotation (Line(points={{143.4,-10},{143.4,-8},{
          133.2,-8}}, color={0,0,127}));
annotation (
  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-180,-120},{160,
            120}}),
          graphics={
        Text(
          extent={{114,-64},{132,-70}},
          lineColor={0,0,255},
          textString=
             "Time"),
        Text(
          extent={{114,-84},{164,-90}},
          lineColor={0,0,255},
          textString=
             "Cooling Power [W]"),
        Text(
          extent={{114,-104},{132,-110}},
          lineColor={0,0,255},
          textString=
             "COP"),
        Text(
          extent={{-98,-110},{-48,-116}},
          lineColor={0,0,255},
          textString=
             "Collected Water [g]")}), Documentation(revisions="<html>
Copyright &copy; 2004-2023, MODELON AB <br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br />This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br />or by other means have their origin from any Modelon Library.
</html>", info="<html>
<p>This model switches the operation of an air conditioner and a heat pump by switching the flow path of the refrigerant with a four-way valve, thereby reversing the flow of the refrigerant in the direction other than the compressor. The heat transfer coefficient of the refrigerant side in the heat exchanger is calculated at a constant value to accommodate both the evaporator and the condenser.</p>
<p>(In general, in air-refrigerant heat exchange, the heat transfer rate of the refrigerant side is so large compared to the heat transfer rate of the air side that the heat transfer rate of the air side is the rate-limiting factor for heat transfer and the effect of the refrigerant side heat transfer rate change on the heat exchanger performance is small.)</p>
<p>The simulation starts in air conditioner mode, and after startHeatCycle seconds, the four-way valve switches to the heat pump cycle.</p>
</html>"),
    experiment(
      StopTime=300,
      __Dymola_NumberOfIntervals=1000,
      Tolerance=1e-06,
      __Dymola_Algorithm="Cvode"),
    __Dymola_experimentFlags(
      Advanced(
        EvaluateAlsoTop=false,
        GenerateVariableDependencies=false,
        OutputModelicaCode=false),
      Evaluate=true,
      OutputCPUtime=true,
      OutputFlatModelica=false),
    Icon(coordinateSystem(extent={{-180,-120},{160,120}})));
end SingleTXV;
