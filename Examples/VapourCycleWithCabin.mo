within Examples;

model VapourCycleWithCabin "Simple cabin connected to a vapour cycle"
  extends .Modelon.Icons.Experiment;
replaceable .AirConditioning.Visualizers.PH_R134a_logp PHDiagram(
  x=h_Diagram,
  y=.Modelica.Math.log(p_Diagram),
  color={255,0,0}) annotation (choicesAllMatching, Placement(transformation(
          extent={{20,20},{100,100}},rotation=0)));

  .Modelica.Units.SI.Pressure[5] p_Diagram={vapourCycle.compressor.summary.p_suction,vapourCycle.compressor.summary.p_discharge,
      vapourCycle.expansionValve.summary.p_in,vapourCycle.expansionValve.summary.p_out,vapourCycle.compressor.summary.p_suction}
    "pressures for p-h diagram";
  .Modelica.Units.SI.SpecificEnthalpy[5] h_Diagram={vapourCycle.compressor.summary.h_suction,vapourCycle.compressor.summary.h_discharge,
      vapourCycle.condenser.summary.h_out,vapourCycle.evaporator.summary.h_in,vapourCycle.compressor.summary.h_suction}
    "specific enthalpies for p-h diagram";
  .AirConditioning.Cycles.VapourCycle vapourCycle(
    redeclare package Medium = Medium,
    expansionValve(
      steadySuperheat=false,
      SuperHeatSetPoint=5),
    evaporator(
      n_segAir=1,
      redeclare model AirModel =
          .AirConditioning.ThermoFluidPro.PipesAndVolumes.HXAirFlowMoistAnalytic,
      redeclare model HTCoefficientRefSide =
          .AirConditioning.ThermoFluidPro.HeatTransfer.HTTwoPhaseMedium.KcSimpleTwoPhase (k_2ph=
              7000.0)),
    condenser(redeclare model HTCoefficientRefSide =
          .AirConditioning.ThermoFluidPro.HeatTransfer.HTTwoPhaseMedium.KcTurbulentFilmCondensation,
        redeclare model RefrigerantFrictionLossModel =
          .AirConditioning.ThermoFluidPro.PressureLoss.PLossHexChannel.DensityProfilePressureLossHX
          (
          h0_in=450e3,
          h0_out=270e3,
          mdot0=0.02,
          p0_in=1960000,
          p0_out=1880000)),
    init(
      mdotCond=airInit.mdotCond,
      mdotEvap=airInit.mdotEvap,
      TairCond=airInit.TairCond,
      TairEvap=airInit.TairEvap,
      phiCond=airInit.phiCond,
      phiEvap=airInit.phiEvap,
      initType=1,
      mdot_init=0.06,
      T_sh=5,
      dp_high=400000,
      p_suction=500000,
      dT_airCond = 18),
    redeclare .AirConditioning.Visualizers.PH_R134a_logp PHDiagram(
      x=h_Diagram,
      y=.Modelica.Math.log(p_Diagram),
      color={255,0,0}))
                      annotation (Placement(transformation(extent={{-52,-26},{
            -6,20}},
                  rotation=0)));
.Modelica.Blocks.Sources.Constant Cond_phi(k=airInit.phiCond)
    "relative humidity"
  annotation (Placement(transformation(extent={{-100,22},{-88,34}},rotation=0)));
.Modelica.Blocks.Sources.Ramp Cond_mair(
  height=0,
  startTime=1.0e6,
    offset=airInit.mdotCond,
    duration=2)             annotation (Placement(transformation(extent={{-96,42},
            {-82,56}},     rotation=0)));
.Modelica.Blocks.Sources.Ramp Cond_tair(
  duration=1,
  height=20,
    startTime=1e6,
    offset=airInit.TairCond)
                   annotation (Placement(transformation(extent={{-90,62},{-76,
            76}}, rotation=0)));
  .AirConditioning.Reservoirs.Air.FlowSourceAir airIn_cond annotation (Placement(
        transformation(extent={{-82,2},{-64,20}},  rotation=0)));
  .AirConditioning.Reservoirs.Air.FlowSinkAir airOut_cond annotation (Placement(
        transformation(extent={{10,-10},{-10,10}},
                                                 rotation=90,
        origin={2,22})));
  .AirConditioning.AirHandling.SimpleCabin simpleCabin(
    n_Passenger=4,
    init(
      p0=1.1e5,
      mdot0=airInit.mdotEvap,
      T0=airInit.TairEvap,
      phi0=airInit.phiEvap),
    redeclare package Medium = .AirConditioning.ThermoFluidPro.Media.Air.NoncondensingAir,
    V=5,
    C=(simpleCabin.P_evaporator/simpleCabin.deltaT)*simpleCabin.timeConstant*
        0.2,
    deltaT=40,
    Tm=318.15)   annotation (Placement(transformation(extent={{-46,-82},{-22,
            -58}}, rotation=0)));
  .AirConditioning.AirHandling.AirDuct airDuct(
    p_nom=1.2e5,
    T_nom=320,
    dp_nom=500,
    d_nom=1.2,
    mdot_nom=airInit.mdotEvap,
    init(
      p0=1.2e5,
      T0=airInit.TairEvap,
      mdot0=airInit.mdotEvap,
      phi0=airInit.phiEvap),
    redeclare package Medium = .AirConditioning.ThermoFluidPro.Media.Air.NoncondensingAir)
                  annotation (Placement(transformation(extent={{-84,-80},{-64,
            -60}}, rotation=0)));
.Modelica.Blocks.Sources.Ramp valveOpening(
    duration=10,
    height=1,
    offset=0.01,
    startTime=1e6)         annotation (Placement(transformation(
        origin={0,-44},
        extent={{-6,6},{6,-6}},
        rotation=270)));
  .AirConditioning.AirHandling.ByPassValve byPassValve(
    T_nom_a=310,
    p_nom_a=0.98e5,
    dp_nom_a=1000,
    T_nom_rec=310,
    p_nom_rec=0.98e5,
    dp_nom_rec=1000,
    maxOpening=0.99,
    minOpening=0.01,
    mdot_nom_a=0.0944,
    mdot_nom_rec=0.0944,
    T_ambient=310,
    phi_ambient=0.6,
    init(
      mdot0=airInit.mdotEvap,
      T0=airInit.TairEvap,
      phi0=airInit.phiEvap,
      p0=97700),
    redeclare package Medium = .AirConditioning.ThermoFluidPro.Media.Air.NoncondensingAir,
    frictionLoss_a1(b(h_outflow(start=1e5))))
                     annotation (Placement(transformation(extent={{-4,-80},{16,
            -60}}, rotation=0)));
  .AirConditioning.Reservoirs.HeatFlow.Heat1Dim heat1Dim(P0=1947, paraOption=
        false) annotation (Placement(transformation(extent={{-44,-54},{-24,-34}},
          rotation=0)));
  .Modelica.Blocks.Sources.Ramp Speed(
    height=0,
    startTime=1.0e6,
    offset=vapourCycle.init.compSpeed,
    duration=2)              annotation (Placement(transformation(extent={{24,-4},
            {12,8}},      rotation=0)));
  replaceable package Medium = .AirConditioning.ThermoFluidPro.Media.Technical.R134a
    constrainedby .AirConditioning.ThermoFluidPro.Media.Interfaces.ExtendedPartialTwoPhaseMedium
    "Refrigerant medium" annotation (choicesAllMatching);

  .AirConditioning.AirHandling.IdealFan blower(
    redeclare package Medium = .AirConditioning.ThermoFluidPro.Media.Air.NoncondensingAir,
    init(
      p0=0.99e5,
      T0=airInit.TairEvap,
      phi0=airInit.phiEvap,
      mdot0=airInit.mdotEvap),
    m_flow=airInit.mdotEvap,
    V_tot=0.001)         annotation (Placement(transformation(extent={{30,-80},
            {50,-60}}, rotation=0)));
  .Modelica.Blocks.Sources.Constant AirMassFlow(k=airInit.mdotEvap)
                                   annotation (Placement(transformation(
        origin={40,-44},
        extent={{-6,6},{6,-6}},
        rotation=270)));
public
  .Modelica.Blocks.Sources.Constant T_ref_Celsius(k=5) "Temperature reference"
    annotation (Placement(transformation(
        origin={54,-8},
        extent={{-6,-6},{6,6}},
        rotation=180)));
  .AirConditioning.ControllersAndSensors.LimPI LimPI_LP(
    steadyStateInit=false,
    yMin=0.1,
    Ti=1,
    k=-0.05,
    yInit=0.999)
            annotation (Placement(transformation(extent={{40,-14},{28,-2}},
          rotation=0)));
  .AirConditioning.ControllersAndSensors.AirTemperature tempSensor(isCelsius=true)
    annotation (Placement(transformation(extent={{-58,-8},{-78,-28}}, rotation=
            0)));
  .AirConditioning.Visualizers.RealValue valueP_evap(precision=0, number=
        vapourCycle.summary.P_evaporator)
                      annotation (Placement(transformation(extent={{-42,34},{
            -16,54}}, rotation=0)));
  .AirConditioning.Visualizers.RealValue valueCOP(                   precision=1,
      number=vapourCycle.summary.COP)
    annotation (Placement(transformation(extent={{-42,74},{-16,94}}, rotation=0)));
.AirConditioning.Visualizers.RealValue valueCondensate(precision=1, number=
        vapourCycle.waterAccumulator.M*1000)          annotation (Placement(
        transformation(extent={{-42,54},{-16,74}}, rotation=0)));
  .AirConditioning.SubComponents.Records.InitData.AirInit airInit(
    mdotCond=0.5,
    TairEvap=58.5 + 273.15,
    TairCond=40 + 273.15,
    mdotEvap=0.095)
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}},
          rotation=0)));
  .AirConditioning.Visualizers.RealValue valueTcabin(precision=1, number=.Modelica.Units.Conversions.to_degC(
        simpleCabin.cabinVolume.T))
    annotation (Placement(transformation(extent={{-42,-102},{-16,-82}}, rotation=0)));
  .AirConditioning.Visualizers.RealValue valueAirflow(
    precision=2, number=blower.a.m_flow)
    annotation (Placement(transformation(extent={{56,-102},{82,-82}}, rotation=
            0)));
  .AirConditioning.Visualizers.RealValue valueTout(
    precision=1, number=tempSensor.outPort)
    annotation (Placement(transformation(extent={{-92,-58},{-66,-38}}, rotation=
           0)));
  .AirConditioning.Visualizers.RealValue valueTin(precision=1, number=.Modelica.Units.Conversions.to_degC(
        vapourCycle.summary.TairEvap_in))
    annotation (Placement(transformation(extent={{70,-54},{96,-34}}, rotation=0)));
  .AirConditioning.Visualizers.RealValue valueTime(precision=1, number=time)
    annotation (Placement(transformation(extent={{70,-22},{96,-2}},rotation=0)));
equation

  connect(Cond_mair.y, airIn_cond.M_flow_in) annotation (Line(
      points={{-81.3,49},{-76.6,49},{-76.6,19.55}},
      color={0,0,127},
      thickness=0.5));
  connect(vapourCycle.cond_air_in, airIn_cond.b) annotation (Line(
      points={{-52,10.8},{-52,11},{-64,11}},
      color={0,127,255},
      thickness=0.5));
  connect(Cond_phi.y, airIn_cond.phi_in) annotation (Line(
      points={{-87.4,28},{-84,28},{-84,15.5},{-81.55,15.5}},
      color={0,0,127},
      thickness=0.5));
  connect(Cond_tair.y, airIn_cond.T_in) annotation (Line(
      points={{-75.3,69},{-70.3,69},{-70.3,19.55}},
      color={0,0,127},
      thickness=0.5));
  connect(airOut_cond.a, vapourCycle.cond_air_out) annotation (Line(
      points={{2,12},{2,10.8},{-6,10.8}},
      color={0,127,255},
      thickness=0.5));
  connect(valveOpening.y, byPassValve.opening)
                                          annotation (Line(
      points={{-1.2124e-15,-50.6},{-1.2124e-15,-55.3},{0,-60},{0,-59.6}},
      color={0,0,127},
      thickness=0.5));
  connect(heat1Dim.q,simpleCabin. q) annotation (Line(points={{-34,-52.8},{-34,
          -59.2}}, color={255,0,0}));
  connect(airDuct.b,simpleCabin. a) annotation (Line(
      points={{-64,-70},{-46,-70}},
      color={0,127,255},
      thickness=0.5));
  connect(byPassValve.b, blower.a)  annotation (Line(
      points={{16,-70},{30,-70}},
      color={0,127,255},
      thickness=0.5));
  connect(blower.b, vapourCycle.evap_air_in[1, 1]) annotation (Line(
      points={{50,-70},{56,-70},{56,-24},{-6,-24},{-6,-16.8}},
      color={0,127,255},
      thickness=0.5));
  connect(T_ref_Celsius.y, LimPI_LP.u_s)        annotation (Line(
      points={{47.4,-8},{41.2,-8}},
      color={0,0,255},
      thickness=0.5));
  connect(LimPI_LP.y, vapourCycle.relative_Vol) annotation (Line(
      points={{27.4,-8},{18,-8},{18,-7.6},{-5.08,-7.6}},
      color={0,0,127},
      thickness=0.5));
  connect(tempSensor.outPort, LimPI_LP.u_m)     annotation (Line(
      points={{-68,-28},{-68,-30},{34,-30},{34,-15.2}},
      color={0,0,127},
      thickness=0.5));
  connect(Speed.y, vapourCycle.comp_speed) annotation (Line(
      points={{11.4,2},{3.16,2},{3.16,1.6},{-5.08,1.6}},
      color={0,0,127},
      thickness=0.5));
  connect(AirMassFlow.y, blower.massFlow) annotation (Line(
      points={{40,-50.6},{40,-62}},
      color={0,0,127},
      thickness=0.5));
  connect(simpleCabin.b, byPassValve.a) annotation (Line(
      points={{-22,-70},{-4,-70}},
      color={0,127,255},
      thickness=0.5));
  connect(tempSensor.b, airDuct.a) annotation (Line(
      points={{-78,-18.9091},{-98,-18.9091},{-98,-70},{-84,-70}},
      color={0,127,255},
      thickness=0.5));
  connect(tempSensor.a, vapourCycle.evap_air_out) annotation (Line(
      points={{-58,-18.9091},{-52,-18.9091},{-52,-16.8}},
      color={0,127,255},
      thickness=0.5));
   annotation (
    Icon(graphics),
    Diagram(graphics={
        Text(
          extent={{-48,56},{2,50}},
          lineColor={0,0,255},
          textString=
               "Cooling Power [W]"),
        Text(
          extent={{-46,96},{-28,90}},
          lineColor={0,0,255},
          textString=
               "COP"),
        Text(
          extent={{-46,76},{0,70}},
          lineColor={0,0,255},
          textString=
             "Collected Water [g]"),
        Text(
          extent={{-42,-76},{4,-90}},
          lineColor={0,0,255},
          textString=
               "Cabin Temperature [C]"),
        Text(
          extent={{50,-76},{90,-88}},
          lineColor={0,0,255},
          textString=
               "Air Mass Flow"),
        Text(
          extent={{-94,-32},{-48,-46}},
          lineColor={0,0,255},
          textString=
               "Temperature at Evap. Outlet [C]"),
        Text(
          extent={{64,-28},{104,-38}},
          lineColor={0,0,255},
          textString="Temperature at 
Evap. Inlet [C]"),
        Text(
          extent={{72,0},{90,-6}},
          lineColor={0,0,255},
          textString=
               "Time")}),
    Documentation(revisions="<html>
Copyright &copy; 2004-2023, MODELON AB <br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br />This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br />or by other means have their origin from any Modelon Library.
</html>",
      info="<html>
<h4>VapourCycleWithCabin</h4>
<p>
This is an example of a <a href=\"modelica://AirConditioning.AirHandling.SimpleCabin\">cabin</a> connected over the evaporator in a <a href=\"modelica://AirConditioning.Cycles.VapourCycle\">vapour cycle</a>. The compressor displacement is controlled to obtain the set-point value of the outlet air temperature from the evaporator.   
</p>
 
<p>
All air boundary conditions are set in the <a href=\"modelica://AirConditioning.SubComponents.Records.InitData.AirInit\">airInit</a> record.
The air mass flow is decided by the <a href=\"modelica://AirConditioning.AirHandling.IdealFan\">blower</a> model which sets a fixed value of the mass flow rate. 
In this example the air is recirculated. To change this so that the air is taken from the ambient environment the <a href=\"modelica://AirConditioning.AirHandling.ByPassValve\">byPassValve</a> model should be used.   
</p>
 <p></p>
</html>"),
    experiment(StopTime=4000, Tolerance=1e-005),
    __Dymola_experimentSetupOutput(
    equidistant=false));
end VapourCycleWithCabin;
