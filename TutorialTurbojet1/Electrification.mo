within TutorialTurbojet1;

model Electrification "Model built in the third tutorial section"

    inner .JetPropulsion.Settings_JPL settings_JPL(usePbS = false,onDesign = true) annotation(Placement(transformation(extent = {{-40.7625,-100.0},{-20.762500000000003,-80.0}},origin = {0.0,0.0},rotation = 0.0)));

    inner .Modelon.Environment.AirDataImplementations.XYZ airData(Mn_par = 0,alt_par = 0,alt_input = true,Mn_input = true) annotation(Placement(transformation(extent = {{-0.21249999999999858,-100.0},{19.7875,-80.0}},origin = {0.0,0.0},rotation = 0.0)));

    inner .Modelon.Environment.AtmosphereImplementations.US76 atmosphere annotation(Placement(transformation(extent = {{33.7375,-100.0},{77.7375,-80.0}},origin = {0.0,0.0},rotation = 0.0)));

    inner .Modelon.Environment.WorldRepresentation worldRepresentation annotation(Placement(transformation(extent = {{99.2375,-100.0},{119.2375,-80.0}},origin = {0.0,0.0},rotation = 0.0)));

    .JetPropulsion.Boundaries.AmbientBoundary ambient(w=19.9581) annotation (Placement(
        transformation(
        extent={{-78.05,0.0},{-58.05,20.0}},
        origin={0.0,0.0},
        rotation=0.0)));

    .JetPropulsion.Basic.Inlets.Inlet inlet(
      redeclare replaceable model RPR =
        .JetPropulsion.Basic.Inlets.RamPressureRecovery.MachNumberDependent,
      useSensor_a = true)
      annotation(Placement(transformation(extent = {{-52.05,0.0},{-32.05,20.0}},origin = {0.0,0.0},rotation = 0.0)));

    .JetPropulsion.Basic.Compressors.Compressor cmp(
      nPorts = 1,
      prDes = 7,
      effPolyPrscr = true,
      redeclare replaceable model RlineMap = .JetPropulsion.Basic.Compressors.RlineMaps.Default (
        redeclare replaceable package Table =
            .JetPropulsion.Compressors.Sections.Examples.JT9D.Components.Maps.HighPressure),
      useSensor_a = true,useSensor_b = true,useSummaryPort = true,
      nBld = 3,  wBldPrscr = true, fracwBld = {0.05,0.025,0}, fracWorkBld = {1,1,1}, fracpBld = {1,1,1})
        annotation(Placement(transformation(extent = {{-20.05,0.0},{-0.05000000000000071,20.0}},origin = {0.0,0.0},rotation = 0.0)));

    .JetPropulsion.Basic.Burners.WithInputPort brn(
      TtCombOutPrscrPar = 1166.483,
      redeclare replaceable model eff =
        .JetPropulsion.Basic.Burners.Correlations.Efficiency.Constant (eff = 0.98),dpqpf = 0.05,
      useSensor_b = true, useInput = true, switchBurn =  .JetPropulsion.Basic.Burners.Internals.SwitchBurn.FAR)
        annotation(Placement(transformation(extent = {{21.95,0.0},{41.95,20.0}},origin = {0.0,0.0},rotation = 0.0)));

    .JetPropulsion.Basic.Turbines.Turbine trb(
      nPorts = 1,
      NDesPrscr = 16500,
      effPolyPrscr = true,
      effPolyDesPrscr = 0.83,
      redeclare replaceable model PrMap = .JetPropulsion.Basic.Turbines.PRmaps.Default (
        redeclare replaceable package Table =
            .JetPropulsion.Turbines.Sections.Examples.JT9D.Components.Maps.HighPressure),
      useSummaryPort = true,
      nBld = 2, turbineCoolingFlow = {.JetPropulsion.Utilities.Types.TurbineCoolingFlow.Full,.JetPropulsion.Utilities.Types.TurbineCoolingFlow.None},
        redeclare replaceable model Cooling =
          .JetPropulsion.Basic.Turbines.Correlations.Cooling.ConvHaselbacher,                                    
       redeclare replaceable model PressureLossBld1 =
          .JetPropulsion.Basic.Turbines.Correlations.PressureLoss.NozzleGuideVane,                                                                                                                                                      
       redeclare replaceable model PressureLossBld3 =
          .JetPropulsion.Basic.Turbines.Correlations.PressureLoss.RotorCooling)
        annotation(Placement(transformation(extent = {{61.95,0.0},{81.95,20.0}},origin = {0.0,0.0},rotation = 0.0)));

    .JetPropulsion.Basic.Nozzles.Nozzle noz(
      redeclare replaceable model Friction =
        .JetPropulsion.Basic.Nozzles.Correlations.Friction.Constant (dpqpPrscr = 0.1),
      useCV = false,
      redeclare replaceable model ConDivThrustCoefficient =
        .JetPropulsion.Basic.Nozzles.Correlations.ThrustCoefficient.ConDiv.Constant(CFgPrscr = 0.98))
        annotation(Placement(transformation(extent = {{101.95,0.0},{121.95,20.0}},origin = {0.0,0.0},rotation = 0.0)));

    .JetPropulsion.CyclePerformance.Control.Burner.FarFromOutletTempAndSpeed farFromT4AndNc(
       TtCombOutPrscrPar = 1166,
       fix = false,
       useConstraint = true,
       useConstraintOnly = false)
      annotation(Placement(transformation(extent = {{-10.0,-10.0},{10.0,10.0}},origin = {90.0,50.0},rotation = -180.0)));

    inner .JetPropulsion.CyclePerformance.Aggregates.Turbojet cycleProperties annotation (Placement(
        transformation(
        extent={{12.0,-64.0},{52.0,-44.0}},
        origin={0.0,0.0},
        rotation=0.0)));    
    
    

    .JetPropulsion.Boundaries.PressureBoundary custBld(
      isSource = false,
      isSimple = false,
      specifypStdy = false,
      specifypDyn = false,
      specifypDes = false,
      specifywDes = false)
        annotation(Placement(transformation(extent = {{-10.0,-10.0},{10.0,10.0}},origin = {30.0,62.0},rotation = -180.0)));
    .Modelica.Blocks.Sources.Ramp alt(duration = 18,startTime = 12,height = 5000) annotation(Placement(transformation(extent = {{-94.78971664236235,-84.78971664236235},{-85.21028335763765,-75.21028335763765}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelica.Blocks.Sources.Ramp Mn(height = 0.7,startTime = 10,duration = 20) annotation(Placement(transformation(extent = {{-74.26886319696908,-94.26886319696908},{-65.73113680303092,-85.73113680303092}},origin = {0.0,0.0},rotation = 0.0)));
    
    
  .Electrification.Machines.Examples.Machine electricMachine(
    redeclare replaceable .Electrification.Machines.Mechanical.IdealShaft mechanical,
    controller(external_torque=true),
    enable_thermal_port=false,
    core(limits(
        enable_P_max=false,
        enable_tau_max=false,
        I_dc_max_mot=10000)),
    redeclare replaceable .Electrification.Machines.Electrical.Ideal electrical)
    annotation (
      Placement(transformation(
        extent={{-12.0,-66.0},{8.0,-46.0}},
        origin={0.0,0.0},
        rotation=0.0)));

  .Electrification.Machines.Control.Signals.tau_ref tau_ref annotation(Placement(transformation(extent = {{-64.0,-48.0},{-56.0,-40.0}},origin = {0.0,0.0},rotation = 0.0)));

  .Modelica.Blocks.Sources.Constant const(k = 1000) annotation(Placement(transformation(extent = {{-100.0,-54.0},{-80.0,-34.0}},origin = {0.0,0.0},rotation = 0.0)));

  .Electrification.Batteries.Examples.Nominal battery(
    ns = 128,
    np = 20,
    enable_thermal_port = false)
    annotation(Placement(transformation(extent = {{-51.47395338371355,-76.0},{-72.52604661628645,-56.0}},origin = {0.0,0.0},rotation = 0.0)));
    
    
equation
  connect(ambient.port, inlet.portA)
    annotation (Line(points={{-57.25,10},{-46.05,10}}, color={255,128,0}));
    connect(inlet.portB,cmp.portA) annotation(Line(points = {{-37.25,10},{-20.05,10}},color = {255,128,0}));
    connect(cmp.portB[1],brn.portA) annotation(Line(points = {{0.75,10},{21.95,10}},color = {255,128,0}));
    connect(brn.portB,trb.portA) annotation(Line(points = {{42.75,10},{61.95,10}},color = {255,128,0}));
    connect(trb.portB[1],noz.port) annotation(Line(points = {{82.75,10},{101.95,10}},color = {255,128,0}));
    connect(cmp.flange,trb.flange) annotation(Line(points = {{-10.05,0},{-10.05,-16},{71.95,-16},{71.95,0}},color = {0,0,0}));

  connect(inlet.sensor_a, cycleProperties.station0) annotation (Line(
      points={{-48.05,0},{-48.05,-40},{11.599999999999998,-40},{11.599999999999998,-49}},
      color={0,0,0},
      pattern=LinePattern.Dash));
  connect(cmp.sensor_a, cycleProperties.station2) annotation (Line(
      points={{-18.05,0},{-18.05,-32},{16.599999999999998,-32},{16.599999999999998,-49}},
      color={0,0,0},
      pattern=LinePattern.Dash));
  connect(cmp.cmpSummary, cycleProperties.cmpIn) annotation (Line(
      points={{-20.25,2},{-26.25,2},{-26.25,-28},{21.6,-28},{21.6,-45}},
      color={0,0,0},
      pattern=LinePattern.Dash));
  connect(cmp.sensor_b, cycleProperties.station3) annotation (Line(
      points={{-2.0500000000000007,0},{-2.0500000000000007,-24.5},{26.6,-24.5},{26.6,-49}},
      color={0,0,0},
      pattern=LinePattern.Dash));
  connect(brn.sensor_b, cycleProperties.station4) annotation (Line(
      points={{39.95,0},{39.95,-24.5},{37.4,-24.5},{37.4,-49}},
      color={0,0,0},
      pattern=LinePattern.Dash));
  connect(trb.trbSummary, cycleProperties.trbIn) annotation (Line(
      points={{61.95,2},{61.95,-32},{42.4,-32},{42.4,-45}},
      color={0,0,0},
      pattern=LinePattern.Dash));       
  connect(cycleProperties.summary, farFromT4AndNc.summary)
    annotation (Line(points={{32,-64},{32,-70},{154,-70},{154,50},{100,50}}, color={255,204,51}));    
    connect(farFromT4AndNc.far,brn.u) annotation(Line(points = {{79,50},{31.950000000000003,50},{31.950000000000003,22}},color = {253,162,40},pattern = LinePattern.Dash));      
    
    connect(cmp.portBld[1],trb.portBld[1]) annotation(Line(points = {{-10.05,20.8},{-10.05,36},{71.75,36},{71.75,19.6}},color = {255,128,0}));
    connect(cmp.portBld[2],trb.portBld[2]) annotation(Line(points = {{-10.05,20.8},{-10.05,26.8},{71.75,26.8},{71.75,19.6}},color = {255,128,0}));
    connect(cmp.portBld[3],custBld.port) annotation(Line(points = {{-10.05,20.8},{-10.05,62},{19.2,62}},color = {255,128,0}));
    connect(alt.y,airData.alt_in) annotation(Line(points = {{-84.73131169340141,-80},{-50,-80},{-50,-74},{-4,-74},{-4,-80.4},{-0.8125,-80.4}},color = {0,0,127}));
    connect(Mn.y,airData.Mn_in) annotation(Line(points = {{-65.30425048333402,-90},{-46,-90},{-46,-76},{-8,-76},{-8,-83.8},{-0.8125000000000071,-83.8}},color = {0,0,127}));

    connect(const.y,tau_ref.u_r) annotation(Line(points = {{-79,-44},{-66,-44}},color = {0,0,127}));
    connect(electricMachine.flange,trb.flange) annotation(Line(points = {{8,-56},{71.95,-56},{71.95,0}},color = {0,0,0}));
    connect(battery.plug_a,electricMachine.plug_a) annotation(Line(points = {{-51.47395338371355,-66},{-31.736976691856775,-66},{-31.736976691856775,-56},{-12,-56}},color = {255,128,0}));
    connect(tau_ref.systemBus,electricMachine.controlBus) annotation(Line(points = {{-56,-44},{-10,-44},{-10,-46}},color = {240,170,40},pattern = LinePattern.Dot));    
    
    annotation (
      Diagram(coordinateSystem(extent = {{-120.0,-100.0},{160.0,100.0}})),
      Icon(coordinateSystem(preserveAspectRatio = false,
      extent = {{-100.0,-100.0},{100.0,100.0}}),
      graphics={Text(lineColor={0,0,255},extent={{-150,150},{150,110}},textString = "%name")}),
      Documentation(revisions="<html>
        <!--COPYRIGHT-->
        </html>"));
end Electrification;
