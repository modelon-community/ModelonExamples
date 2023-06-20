within Examples;

model TwinEvaporatorCycle
  "Vapor compression cycle with two parallel evaporators"
  .AirConditioning.SubComponents.ComponentSummaries.CycleSummary summary(
    final P_evaporator=evaporator.summary.P_evaporator,
    final P_compressor=compressor.summary.P_shaft,
    final P_condenser=condenser.summary.P_condenser,
    final COP=(evaporator.summary.P_evaporator + rearEvaporator.summary.P_evaporator)
        /max(1e-5, summary.P_compressor),
    final p_high=compressor.summary.p_discharge,
    final p_low=compressor.summary.p_suction,
    final Superheat=superHeat.TempSH,
    final Subcooling=condenser.summary.Subcooling,
    final TairEvap_in=evaporator.summary.Tair_in,
    final TairEvap_out=evaporator.summary.Tair_out,
    final TairCond_in=condenser.summary.Tair_in,
    final TairCond_out=condenser.summary.Tair_out,
    final h_evap_out=evaporator.summary.h_out,
    final h_evap_in=evaporator.summary.h_in,
    final h_cond_out=condenser.summary.h_out,
    final h_cond_in=condenser.summary.h_in,
    final mdot=compressor.summary.mdot,
    final dp_evap=evaporator.summary.dp_ref,
    final dp_cond=condenser.summary.dp_ref,
    final recLevel=receiver.summary.relLevel,
    final recQuality=receiver.summary.quality,
    final M_ref=condenser.summary.M_ref + pipe1.summary.M_ref + pipe2.summary.M_ref
         + pipe3.summary.M_ref + receiver.summary.M_ref + orificeTube.summary.M_ref
         + expansionValve.summary.M_ref + evaporator.summary.M_ref +
        rearEvaporator.summary.M_ref + pipe4.summary.M_ref + compressor.summary.M_ref
         + pipe6.summary.M_ref + pipe7.summary.M_ref + pipe8.summary.M_ref +
        split.volumeA.M[1] + junction.volumeAB.M[1],
    final V_ref=condenser.summary.V_ref + pipe1.summary.V_ref + pipe2.summary.V_ref
         + pipe3.summary.V_ref + receiver.summary.V_ref + orificeTube.summary.V_ref
         + expansionValve.summary.V_ref + rearEvaporator.summary.V_ref +
        evaporator.summary.V_ref + pipe4.summary.V_ref + compressor.summary.V_ref
         + pipe6.summary.V_ref + pipe7.summary.V_ref + pipe8.summary.V_ref +
        split.volumeA.V[1] + junction.volumeAB.V[1]);
extends .AirConditioning.SubComponents.Records.TemplateData.TestbenchPars;
extends .Modelon.Icons.Experiment;
  .AirConditioning.SubComponents.Records.InitData.CycleInit init(
    redeclare package Medium = Medium,
    charge=summary.SpecificCharge,
    M_receiver=receiver.summary.M_ref,
    p_receiver=receiver.summary.p,
    V_receiver=receiver.summary.V_ref,
    V_ref=summary.V_ref,
    charge_init=500,
    initType=2,
    dp_pipe(displayUnit="Pa") = 1000,
    mdotCond=0.7,
    T_sh=7,
    mdotEvap=0.1,
    compSpeed=35,
    steadyState=true,
    steadyPressure=true,
    mdot_init=0.15,
    T_sc=5,
    p_high=2600000,
    dp_high=100000,
    p_suction=400000,
    TairCond=314,
    TairEvap=314,dT_airEvap = -20,dT_airCond = 20)     annotation (Placement(transformation(extent={{112,-50},
            {144,-30}},rotation=0)));
replaceable .AirConditioning.HeatExchangers.Condenser condenser(
  redeclare package Medium = Medium,
  final HX_Init=init.condInit,
    n_segRef=3)
      annotation (choicesAllMatching, Placement(transformation(extent={{-10,74},
            {-50,114}}, rotation=0)));

.AirConditioning.PipesAndVolumes.PipeAdiabatic pipe1(
  n=1,
  redeclare package Medium = Medium,
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
          mdot0=0.15))
                 annotation (Placement(transformation(
        origin={44,-10},
        extent={{-10,-10},{10,10}},
        rotation=180)));
      //n_m=1,

.AirConditioning.PipesAndVolumes.PipeAdiabatic pipe2(
  n=1,
  redeclare package Medium = Medium,
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
        .AirConditioning.ThermoFluidPro.PressureLoss.PressureLossTD (d0=fill(1200.0, pipe2.n),
          mdot0=0.15))
  annotation (Placement(transformation(extent={{-80,70},{-100,90}},rotation=0)));
.AirConditioning.PipesAndVolumes.PipeAdiabatic pipe3(
  n=1,
  redeclare package Medium = Medium,
  geo(D=0.008, L=0.3),
    init(
      steadyState=init.steadyState,
      steadyPressure=init.steadyPressure,
      h_in=init.h_cond_out,
      h_out=init.h_cond_out,
      p_out=init.p_high - init.dp_high - 3*init.dp_pipe,
      p_in=init.p_high - init.dp_high - 2*init.dp_pipe - 2*init.dp_pipe/5,
      mdot0=init.mdot_init/2),
    redeclare replaceable model FrictionLoss =
        .AirConditioning.ThermoFluidPro.PressureLoss.PressureLossTD (d0=fill(1200.0, pipe3.n),
          mdot0=0.15))
                 annotation (Placement(transformation(
        origin={-116,24},
        extent={{10,-10},{-10,10}},
        rotation=90)));
.AirConditioning.PipesAndVolumes.PipeAdiabatic pipe4(
  n=1,
  redeclare package Medium = Medium,
  geo(D=0.015, L=0.4),
    init(
      steadyState=init.steadyState,
      steadyPressure=init.steadyPressure,
      mdot0=init.mdot_init,
      h_in=init.h_suction,
      h_out=init.h_suction,
      p_out=init.p_suction + init.dp_pipe/10,
      p_in=init.p_suction + init.dp_pipe*0.4),
    redeclare replaceable model FrictionLoss =
        .AirConditioning.ThermoFluidPro.PressureLoss.PressureLossTD (
        d0=fill(12.0, pipe4.n),
        dp0=10000,
        mdot0=0.15))
                 annotation (Placement(transformation(
        origin={62,-66},
        extent={{-10,-10},{10,10}},
        rotation=90)));
.AirConditioning.ControllersAndSensors.SuperHeatSensor superHeat(redeclare
      package Medium=Medium)
  annotation (Placement(transformation(extent={{-4,-16},{16,-36}}, rotation=0)));
.Modelica.Blocks.Sources.Ramp Speed(
  final offset=init.compSpeed,
    duration=10,
    height=20,
    startTime=1e6)         annotation (Placement(transformation(extent={{6,-6},{
            -6,6}},   rotation=90,
        origin={80,-6})));
  replaceable .AirConditioning.HeatExchangers.Evaporator evaporator(
    redeclare package Medium = Medium,
    final HX_Init=init.evapInit,
    redeclare model AirModel =
        .AirConditioning.ThermoFluidPro.PipesAndVolumes.HXAirFlowMoistAnalytic,
    n_segAir=1,
    n_segRef=2)
    annotation (choicesAllMatching, Placement(transformation(extent={{-74,-30},
            {-34,10}}, rotation=0)));

  .AirConditioning.Compressors.GenericExternalControl compressor(
    redeclare package Medium = Medium,
    init(
      final h0=init.h_suction,
      final p0=init.p_suction,
      dp0 = init.p_high - init.p_suction,
      steadyState=false,
      steadyPressure=false),
    ce(
      etaEI_a={0.91,-0.0035},
      etaEI_b={2.3,-0.033,3.00E-04},
      etaEI_k={8.5,0.015},
      etaEI_c={1.2,0.002},
      etaEV_pi0=40,
      etaEV_x0=0.012,
      etaEV_a={1,-0.0025,-3e-05},
      etaEI_pi0=40,
      etaEI_x0=0.012,
      etaIC_pi0=40,
      etaIC_x0=0.012,
      etaIC_a={1.1,-0.002},
      etaIC_b={1.8,-0.02,1.40E-04},
      etaIC_k={10,-0.03},
      etaIC_c={1.2,0.005}),
    MaximumDisplacement=210e-6) "simple parameterization for R134a compressor"
  annotation (Placement(transformation(
        origin={62,-34},
        extent={{-10,10},{10,-10}},
        rotation=90)));
.AirConditioning.ThermoFluidPro.Components.Compressors.Speed rps(
  phi(fixed=true, start=0.0),
  exact=true) "rotational speed of compressor"
  annotation (Placement(transformation(extent={{6,-8},{-6,8}},   rotation=90,
        origin={80,-28})));
.Modelica.Blocks.Sources.Constant RelVol(k=1.0) "relative displacement volume"
                                 annotation (Placement(transformation(extent={{96,-56},
            {84,-44}},         rotation=0)));
  .AirConditioning.Valves.SimpleTXV expansionValve(
    redeclare package Medium = Medium,
    yMax=0.12,
    yMin=0.001,
    geo(D=0.0075, L=0.1),
    yInit=0.05,
    SuperHeatSetPoint=init.T_sh,
    init(
      h0=init.h_cond_out,
      p0=init.p_high - init.dp_high - 3*init.dp_pipe - init.dp_pipe/10,
      steadyState=false,
      steadyPressure=false,
        dp0 = init.p_high - init.dp_high - 5 * init.dp_pipe - 2 * init.dp_pipe / (10) - init.dp_low - init.p_suction),
    k=-0.01,
    Ti=20,
    mdot0=init.mdot_init/2,
    steadySuperheat=true)
              annotation (Placement(transformation(
        origin={-116,-8},
        extent={{-10,10},{10,-10}},
        rotation=90)));
  .AirConditioning.Receivers.SimpleSeparator receiver(
    H=0.15,
    H_Out=0.01,
    H_des=0.08,
    porosity_des=0.8,
    zeta=200,
    Di=0.05528,
    redeclare package Medium = Medium,
    desiccant=false,
    init(
      h0=init.h_cond_out,
      p0=init.p_high - init.dp_high - 2*init.dp_pipe - init.dp_pipe/10,
      steadyState=true,
      steadyPressure=true,dp0 = init.dp_pipe / (10)))
                annotation (Placement(transformation(extent={{-106,62},{-126,82}},
          rotation=0)));
  .AirConditioning.Reservoirs.Air.FlowSourceAir airIn_evap annotation (Placement(
        transformation(
        origin={-17,9},
        extent={{-9,9},{9,-9}},
        rotation=180)));
  .AirConditioning.Reservoirs.Air.FlowSourceAir airIn_cond annotation (Placement(
        transformation(extent={{-74,98},{-56,116}}, rotation=0)));
  .AirConditioning.Reservoirs.Air.FlowSinkAir airOut_cond annotation (Placement(
        transformation(
        origin={-6,124},
        extent={{-8,8},{8,-8}},
        rotation=270)));
  .AirConditioning.Reservoirs.Air.FlowSinkAir airOut_evap annotation (Placement(
        transformation(
        origin={-94,24},
        extent={{8,-8},{-8,8}},
        rotation=90)));
.Modelica.Blocks.Sources.Ramp Cond_tair(
  duration=1,
  final offset=init.TairCond,
  startTime=90,
    height=10)     annotation (Placement(transformation(extent={{-36,120},{-50,
            134}}, rotation=0)));
.Modelica.Blocks.Sources.Ramp Cond_mair(
  height=0,
  final offset=init.mdotCond,
  startTime=1.0e6,
    duration=2)                  annotation (Placement(transformation(extent={{-92,120},
            {-78,134}},          rotation=0)));
.Modelica.Blocks.Sources.Constant Cond_phi(k=init.phiCond) "relative humidity"
  annotation (Placement(transformation(extent={{-110,100},{-98,112}},rotation=0)));
.Modelica.Blocks.Sources.Ramp Evap_tair(
  height=0,
  final offset=init.TairEvap,
  startTime=1.0e6)              annotation (Placement(transformation(
        origin={-29,29},
        extent={{7,7},{-7,-7}},
        rotation=180)));
.Modelica.Blocks.Sources.Ramp Evap_mair(
  duration=1,
  height=-0.05,
  final offset=init.mdotEvap,
    startTime=1e6)
                annotation (Placement(transformation(extent={{8,30},{-6,44}},
          rotation=0)));
.Modelica.Blocks.Sources.Constant Evap_phi(k=init.phiEvap) "relative humidity"
  annotation (Placement(transformation(
        origin={12,14},
        extent={{-6,6},{6,-6}},
        rotation=180)));
  .AirConditioning.Receivers.WaterAccumulator waterAccumulator
  annotation (Placement(transformation(extent={{-58,-48},{-44,-34}}, rotation=0)));
.AirConditioning.Visualizers.RealValue valueTime(precision=1, number=time)
  annotation (Placement(transformation(extent={{116,-74},{142,-54}}, rotation=0)));
.AirConditioning.Visualizers.RealValue valueP_evap(precision=0, number=
        summary.P_evaporator)
                    annotation (Placement(transformation(extent={{-70,12},{-44,
            32}}, rotation=0)));
.AirConditioning.Visualizers.RealValue valueCOP(                   precision=1,
      number=summary.COP)
  annotation (Placement(transformation(extent={{116,-96},{142,-76}}, rotation=0)));
.AirConditioning.Visualizers.RealValue valueCondensate(precision=1, number=
        waterAccumulator.M*1000)
                               annotation (Placement(transformation(extent={{
            -90,-54},{-64,-34}}, rotation=0)));
  replaceable .AirConditioning.Visualizers.PH_R134a_logp PHDiagram(
    x=h_Diagram,
    y=.Modelica.Math.log(p_Diagram),
    color={255,0,0}) constrainedby .Modelon.Visualizers.XYDiagramBackgroundImage(
    x=h_Diagram,
    y=.Modelica.Math.log(p_Diagram),
    color={255,0,0})
                 annotation (choicesAllMatching, Placement(transformation(
          extent={{32,32},{148,148}}, rotation=0)));
protected
  .Modelica.Units.SI.Pressure[5] p_Diagram={compressor.summary.p_suction,compressor.summary.p_discharge,
      expansionValve.summary.p_in,expansionValve.summary.p_out,compressor.summary.p_suction}
    "pressures for p-h diagram";
  .Modelica.Units.SI.SpecificEnthalpy[5] h_Diagram={compressor.summary.h_suction,compressor.summary.h_discharge,
      expansionValve.valve.h,evaporator.summary.h_in,compressor.summary.h_suction}
    "specific enthalpies for p-h diagram";
protected
  .AirConditioning.Visualizers.FourValueLegend legend
                                   annotation (Placement(transformation(extent={{112,-22},
            {144,2}},             rotation=0)));
  parameter .Modelica.Units.SI.Time startTime(fixed=false);
public
  .AirConditioning.PipesAndVolumes.Split split(
    init(p0=init.p_high - init.dp_high - 2 * init.dp_pipe - 2 * init.dp_pipe / (10), h0=
          init.h_cond_out,dp0 = init.dp_pipe / (10)),
    redeclare package Medium = Medium,
    D_a=0.008,
    D_b=0.008,
    D_c=0.008) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=90,
        origin={-138,52})));
  replaceable .AirConditioning.HeatExchangers.Evaporator rearEvaporator(
    redeclare package Medium = Medium,
    redeclare record Geometry =
        .AirConditioning.SubComponents.Records.GeometryData.EvaporatorExample (
        flattubes={5,7,5,5,7,5},
        depth=0.055,
        height=0.11),
    redeclare model AirModel =
        .AirConditioning.ThermoFluidPro.PipesAndVolumes.HXAirFlowMoistAnalytic,
    n_segAir=1,
    final HX_Init=init.evapInit,
    n_segRef=2)
    annotation (choicesAllMatching, Placement(transformation(extent={{-76,-128},
            {-36,-88}}, rotation=0)));
  .AirConditioning.Receivers.WaterAccumulator waterAccumulator1
  annotation (Placement(transformation(extent={{-62,-144},{-48,-130}}, rotation=
           0)));
  .AirConditioning.Valves.OrificeTube orificeTube(
    redeclare package Medium = Medium,
    init(
      h0=init.h_cond_out,
      p0=init.p_high - init.dp_high - 3*init.dp_pipe - init.dp_pipe/10,
      dp0 = init.p_high - init.dp_high - 5 * init.dp_pipe - 2 * init.dp_pipe / (10) - init.dp_low - init.p_suction,
      steadyState=false,
      steadyPressure=false),
    mdot0=init.mdot_init/2,
    Ltube=10.0e-3,
    Dtube=0.0013)
                 annotation (Placement(transformation(
        origin={-138,-8},
        extent={{-10,-10},{10,10}},
        rotation=90)));
.AirConditioning.PipesAndVolumes.PipeAdiabatic pipe6(
  n=1,
  redeclare package Medium = Medium,
    init(
      steadyState=init.steadyState,
      steadyPressure=init.steadyPressure,
      h_in=init.h_suction,
      h_out=init.h_suction,
      p_in=init.p_suction + init.dp_pipe,
      p_out=init.p_suction + init.dp_pipe*0.5,
      mdot0=init.mdot_init/2),
    geo(L=0.3, D=0.015),
    redeclare replaceable model FrictionLoss =
        .AirConditioning.ThermoFluidPro.PressureLoss.PressureLossTD (mdot0=0.15, d0=fill(12.0,
            pipe6.n)))
                 annotation (Placement(transformation(
        origin={28,-122},
        extent={{10,10},{-10,-10}},
        rotation=180)));
.AirConditioning.PipesAndVolumes.PipeAdiabatic pipe7(
  n=1,
  redeclare package Medium = Medium,
    init(
      steadyState=init.steadyState,
      steadyPressure=init.steadyPressure,
      h_in=init.h_suction,
      h_out=init.h_suction,
      p_in=init.p_suction + init.dp_pipe,
      p_out=init.p_suction + init.dp_pipe*0.5,
      mdot0=init.mdot_init/2),
    geo(L=0.3, D=0.015),
    redeclare replaceable model FrictionLoss =
        .AirConditioning.ThermoFluidPro.PressureLoss.PressureLossTD (mdot0=0.15, d0=fill(12.0,
            pipe7.n)))
                 annotation (Placement(transformation(
        origin={30,-26},
        extent={{10,10},{-10,-10}},
        rotation=180)));
  .AirConditioning.PipesAndVolumes.Junction junction(
    redeclare package Medium = Medium,
    init(h0=init.h_suction, p0=init.p_suction + init.dp_pipe*0.5),
    zetaC=10,
    D_a=0.015,
    D_b=0.015,
    D_c=0.015)
    annotation (Placement(transformation(
        origin={44,-82},
        extent={{-10,-10},{10,10}},
        rotation=90)));
.AirConditioning.PipesAndVolumes.PipeAdiabatic pipe8(
  n=1,
  redeclare package Medium = Medium,
    init(
      steadyState=init.steadyState,
      steadyPressure=init.steadyPressure,
      h_in=init.h_cond_out,
      h_out=init.h_cond_out,
      p_out=init.p_high - init.dp_high - 3*init.dp_pipe,
      p_in=init.p_high - init.dp_high - 2*init.dp_pipe - 2*init.dp_pipe/5,
      mdot0=init.mdot_init/2),
    geo(D=0.008, L=1.3),
    redeclare replaceable model FrictionLoss =
        .AirConditioning.ThermoFluidPro.PressureLoss.PressureLossTD (mdot0=0.15, d0=fill(1200.0,
            pipe8.n)))
                 annotation (Placement(transformation(
        origin={-138,24},
        extent={{10,-10},{-10,10}},
        rotation=90)));
  .AirConditioning.Reservoirs.Air.FlowSinkAir airOut_evap1 annotation (Placement(
        transformation(
        origin={-94,-72},
        extent={{8,-8},{-8,8}},
        rotation=90)));
  .AirConditioning.Reservoirs.Air.FlowSourceAir airIn_evap1 annotation (
      Placement(transformation(
        origin={-5,-95},
        extent={{-9,9},{9,-9}},
        rotation=180)));
.Modelica.Blocks.Sources.Ramp rEvap_tair(
  height=0,
  final offset=init.TairEvap,
  startTime=1.0e6,
    duration=2)                 annotation (Placement(transformation(
        origin={-21,-75},
        extent={{7,7},{-7,-7}},
        rotation=180)));
.Modelica.Blocks.Sources.Ramp rEvap_mair(
  duration=1,
  height=-0.05,
    startTime=1e6,
    final offset=init.mdotEvap*0.5)
                annotation (Placement(transformation(extent={{20,-74},{6,-60}},
          rotation=0)));
.Modelica.Blocks.Sources.Constant rEvap_phi(
                                          k=init.phiEvap) "relative humidity"
  annotation (Placement(transformation(
        origin={20,-90},
        extent={{-6,6},{6,-6}},
        rotation=180)));
.AirConditioning.Visualizers.RealValue valueCondensate1(
                                                     precision=1, number=
        waterAccumulator1.M*1000)
                               annotation (Placement(transformation(extent={{
            -92,-154},{-66,-134}}, rotation=0)));
.AirConditioning.Visualizers.RealValue valueP_evap1(
                                                 precision=0, number=
        rearEvaporator.summary.P_evaporator)
                    annotation (Placement(transformation(extent={{-70,-86},{-44,
            -66}}, rotation=0)));
  .AirConditioning.ControllersAndSensors.MultiDisplaySensor multiDisplaySensor(redeclare package
      Medium = Medium)
    annotation (Placement(transformation(extent={{13,13},{-13,-13}},
        rotation=270,
        origin={7,61})));
  .AirConditioning.ControllersAndSensors.MultiDisplaySensor multiDisplaySensor1(redeclare
      package Medium = Medium)
    annotation (Placement(transformation(extent={{-56,90},{-76,70}})));
  .AirConditioning.ControllersAndSensors.MultiDisplaySensor multiDisplaySensor2(redeclare
      package Medium = Medium)
    annotation (Placement(transformation(extent={{-28,-16},{-8,-36}})));
  .AirConditioning.ControllersAndSensors.MultiDisplaySensor multiDisplaySensor3(redeclare
      package Medium = Medium)
    annotation (Placement(transformation(extent={{-18,-112},{2,-132}})));
  .AirConditioning.ControllersAndSensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot
    annotation (Placement(transformation(extent={{10,-154},{-26,-118}})));
  .AirConditioning.ControllersAndSensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot1
    annotation (Placement(transformation(extent={{0,-60},{-36,-24}})));
  .AirConditioning.ControllersAndSensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot2
    annotation (Placement(transformation(extent={{-6,42},{-42,78}})));
  .AirConditioning.ControllersAndSensors.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot3
    annotation (Placement(transformation(extent={{-48,40},{-84,76}})));
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
connect(pipe4.b, compressor.SuctionPort)
  annotation (Line(
      points={{62,-56},{62,-44}},
      color={0,191,0},
      thickness=0.5));
connect(rps.flange,  compressor. Flange_a1)
  annotation (Line(points={{80,-34},{71,-34}},                   color={0,0,0}));
connect(Speed.y,       rps.inPort) annotation (Line(
      points={{80,-12.6},{80,-20.8}},
      color={0,0,255},
      thickness=0.5));
connect(RelVol.y,      compressor. relative_Vol) annotation (Line(
      points={{83.4,-50},{69.5,-50},{69.5,-43.5}},
      color={0,0,255},
      thickness=0.5));
connect(expansionValve.b, evaporator.a1)
                              annotation (Line(
      points={{-116,-18},{-116,-26},{-74,-26},{-74,-24.4}},
      color={0,191,0},
      thickness=0.5));
connect(superHeat.outPort, expansionValve.DeltaT_SH)
  annotation (Line(
      points={{6,-36},{6,-54},{-130,-54},{-130,-14.5},{-125.5,-14.5}},
      color={0,0,255},
      thickness=0.5));
connect(pipe2.b, receiver.FlowIn)      annotation (Line(
      points={{-100,80},{-100,79},{-107.4,79}},
      color={0,191,0},
      thickness=0.5));
connect(pipe3.b, expansionValve.a)
  annotation (Line(
      points={{-116,14},{-116,2}},
      color={0,191,0},
      thickness=0.5));
connect(pipe1.a, compressor.DischargePort)
  annotation (Line(
      points={{54,-10},{62,-10},{62,-24}},
      color={0,191,0},
      thickness=0.5));
connect(evaporator.water_out, waterAccumulator.w_in) annotation (Line(
      points={{-44,-27.6},{-44,-38.2},{-45.4,-38.2}},
      color={0,0,255},
      thickness=0.5));
connect(airIn_cond.b, condenser.air_in) annotation (Line(
      points={{-56,107},{-50,107},{-50,106}},
      color={0,127,255},
      thickness=0.5));
connect(airIn_cond.T_in,Cond_tair.y)        annotation (Line(
      points={{-62.3,115.55},{-62,115.55},{-62,127},{-50.7,127}},
      color={0,0,255},
      thickness=0.5));
connect(airOut_evap.a, evaporator.air_out) annotation (Line(
      points={{-94,16},{-94,2},{-74,2}},
      color={0,127,255},
      thickness=0.5));
connect(airIn_evap.b, evaporator.air_in) annotation (Line(
      points={{-26,9},{-26,8},{-34,8},{-34,2}},
      color={0,127,255},
      thickness=0.5));
connect(Evap_tair.y,       airIn_evap.T_in) annotation (Line(
      points={{-21.3,29},{-19.7,29},{-19.7,17.55}},
      color={0,0,255},
      thickness=0.5));
connect(Evap_phi.y,       airIn_evap.phi_in) annotation (Line(
      points={{5.4,14},{-8,14},{-8,13.5},{-8.45,13.5}},
      color={0,0,255},
      thickness=0.5));
connect(airOut_cond.a, condenser.air_out) annotation (Line(
      points={{-6,116},{-6,106},{-10,106}},
      color={0,127,255},
      thickness=0.5));
connect(Cond_phi.y,       airIn_cond.phi_in) annotation (Line(
      points={{-97.4,106},{-86,106},{-86,111.5},{-73.55,111.5}},
      color={0,0,255},
      thickness=0.5));
connect(Cond_mair.y,       airIn_cond.M_flow_in) annotation (Line(
      points={{-77.3,127},{-68.6,127},{-68.6,115.55}},
      color={0,0,255},
      thickness=0.5));
connect(Evap_mair.y,       airIn_evap.M_flow_in) annotation (Line(
      points={{-6.7,37},{-13.4,37},{-13.4,17.55}},
      color={0,0,255},
      thickness=0.5));
  connect(split.a, receiver.FlowOut) annotation (Line(
      points={{-138,62},{-138,79},{-125,79}},
      color={0,191,0},
      thickness=0.5));
  connect(pipe3.a, split.c) annotation (Line(
      points={{-116,34},{-116,52},{-128,52}},
      color={0,191,0},
      thickness=0.5));
  connect(waterAccumulator1.w_in, rearEvaporator.water_out) annotation (Line(
      points={{-49.4,-134.2},{-46,-134.2},{-46,-125.6}},
      color={0,0,255},
      thickness=0.5));
  connect(superHeat.b, pipe7.a) annotation (Line(
      points={{16,-26},{20,-26}},
      color={0,191,0},
      thickness=0.5));
  connect(pipe7.b, junction.b) annotation (Line(
      points={{40,-26},{44,-26},{44,-72}},
      color={0,191,0},
      thickness=0.5));
  connect(pipe6.b, junction.a) annotation (Line(
      points={{38,-122},{44,-122},{44,-92}},
      color={0,191,0},
      thickness=0.5));
  connect(junction.c, pipe4.a) annotation (Line(
      points={{54,-82},{62,-82},{62,-76}},
      color={0,191,0},
      thickness=0.5));
  connect(orificeTube.a, pipe8.b) annotation (Line(
      points={{-138,2},{-138,14}},
      color={0,191,0},
      thickness=0.5));
  connect(split.b, pipe8.a) annotation (Line(
      points={{-138,42},{-138,34}},
      color={0,191,0},
      thickness=0.5));
  connect(orificeTube.b, rearEvaporator.a1) annotation (Line(
      points={{-138,-18},{-138,-122},{-76,-122},{-76,-122.4}},
      color={0,191,0},
      thickness=0.5));
  connect(airOut_evap1.a, rearEvaporator.air_out) annotation (Line(
      points={{-94,-80},{-94,-96},{-76,-96}},
      color={0,127,255},
      thickness=0.5));
  connect(rEvap_tair.y, airIn_evap1.T_in)   annotation (Line(
      points={{-13.3,-75},{-7.7,-75},{-7.7,-86.45}},
      color={0,0,255},
      thickness=0.5));
  connect(rEvap_mair.y, airIn_evap1.M_flow_in)   annotation (Line(
      points={{5.3,-67},{-1.4,-67},{-1.4,-86.45}},
      color={0,0,255},
      thickness=0.5));
  connect(airIn_evap1.b, rearEvaporator.air_in) annotation (Line(
      points={{-14,-95},{-18,-95},{-18,-96},{-36,-96}},
      color={0,127,255},
      thickness=0.5));
  connect(rEvap_phi.y, airIn_evap1.phi_in) annotation (Line(
      points={{13.4,-90},{12,-90},{12,-90.5},{3.55,-90.5}},
      color={0,0,127},
      thickness=0.5));
  connect(condenser.a1, multiDisplaySensor.portB) annotation (Line(points={{-10,
          79.6},{-10,78},{7,78},{7,74}},    color={0,191,0},
      thickness=0.5));
  connect(multiDisplaySensor.portA, pipe1.b)
    annotation (Line(points={{7,48},{24,48},{24,-10},{34,-10}},
                                                       color={0,191,0},
      thickness=0.5));
  connect(condenser.b1, multiDisplaySensor1.portA) annotation (Line(points={{-50,
          79.6},{-54,79.6},{-54,80},{-56,80}},
                                          color={0,191,0},
      thickness=0.5));
  connect(pipe2.a, multiDisplaySensor1.portB)
    annotation (Line(points={{-80,80},{-76,80}}, color={0,191,0},
      thickness=0.5));
  connect(evaporator.b1, multiDisplaySensor2.portA) annotation (Line(points={{
          -34,-24.4},{-30,-24.4},{-30,-26},{-28,-26}}, color={0,191,0},
      thickness=0.5));
  connect(superHeat.a, multiDisplaySensor2.portB)
    annotation (Line(points={{-4,-26},{-8,-26}}, color={0,191,0},
      thickness=0.5));
  connect(rearEvaporator.b1, multiDisplaySensor3.portA) annotation (Line(points={{-36,
          -122.4},{-26,-122.4},{-26,-122},{-18,-122}},      color={0,191,0},
      thickness=0.5));
  connect(pipe6.a, multiDisplaySensor3.portB) annotation (Line(points={{18,-122},
          {2,-122}},                     color={0,191,0},
      thickness=0.5));
  connect(multiDisplaySensor3.u, multiDisplayVis_phTmdot.y)
    annotation (Line(points={{-8,-122},{-8,-136}}, color={0,0,0},
      thickness=0.5));
  connect(multiDisplaySensor2.u, multiDisplayVis_phTmdot1.y)
    annotation (Line(points={{-18,-26},{-18,-42}}, color={0,0,0},
      thickness=0.5));
  connect(multiDisplaySensor.u, multiDisplayVis_phTmdot2.y)
    annotation (Line(points={{7,61},{-8.5,61},{-8.5,60},{-24,60}},
                                                       color={0,0,0},
      thickness=0.5));
  connect(multiDisplaySensor1.u, multiDisplayVis_phTmdot3.y)
    annotation (Line(points={{-66,80},{-66,58}}, color={0,0,0},
      thickness=0.5));
annotation (
  Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-150,-150},{150,150}},
        initialScale=0.1), graphics={
        Text(
          extent={{112,-52},{130,-58}},
          lineColor={0,0,255},
          textString=
             "Time"),
        Text(
          extent={{-82,34},{-32,28}},
          lineColor={0,0,255},
          textString=
             "Cooling Power [W]"),
        Text(
          extent={{112,-74},{130,-80}},
          lineColor={0,0,255},
          textString=
             "COP"),
        Text(
          extent={{-100,-34},{-50,-40}},
          lineColor={0,0,255},
          textString=
             "Collected Water [g]"),
        Text(
          extent={{-106,-134},{-56,-140}},
          lineColor={0,0,255},
          textString=
             "Collected Water [g]"),
        Text(
          extent={{-82,-64},{-32,-70}},
          lineColor={0,0,255},
          textString=
             "Cooling Power [W]")}),
  Documentation(info="<html>
<body>
<h4>TwinEvaporatorCycle</h4>
<p>
Example of a 2 evaporator system. The cycle summary variables all refer to the front evaporator, for the back evaporator, please refer to the summary of the component.
</p>
<p>After 90s of simulation there is a change as the air temperature over the condenser is incresed.</p>
 
<p></p>
</body>
</html>", revisions="<html>
Copyright &copy; 2004-2023, MODELON AB <br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br />This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br />or by other means have their origin from any Modelon Library.
</html>"),
  experiment(
    StopTime=180,
    Tolerance=1e-006),
  __Dymola_experimentSetupOutput(
    equidistant=true,
    derivatives=false));
end TwinEvaporatorCycle;
