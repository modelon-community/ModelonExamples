within TutorialTurbojet1;

model DifferentConditions "Model built in the second tutorial section"

    inner .JetPropulsion.Settings_JPL settings_JPL(usePbS = true,onDesign = true) annotation(Placement(transformation(extent = {{-40.7625,-100.0},{-20.762500000000003,-80.0}},origin = {0.0,0.0},rotation = 0.0)));

    inner .Modelon.Environment.AirDataImplementations.XYZ airData(Mn_par = 0,alt_par = 0) annotation(Placement(transformation(extent = {{-0.21249999999999858,-100.0},{19.7875,-80.0}},origin = {0.0,0.0},rotation = 0.0)));

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
      useSensor_a = true,useSensor_b = true,useSummaryPort = true)
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
      useSummaryPort = true)
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
    
    
  // Aggregate Properties
  Real NetThrust_Fg_US=Modelon.Units.Conversions.to_lbf(cycleProperties.Fn);
  Real NetFuelFlowRate_Wfuel_US=Modelon.Units.Conversions.to_lbmPerS(cycleProperties.wFuel);
  Real NetSFC_SFC_US=cycleProperties.SFC;


    
    
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
    
    annotation (
      Diagram(coordinateSystem(extent = {{-120.0,-100.0},{160.0,100.0}})),
      Icon(coordinateSystem(preserveAspectRatio = false,
      extent = {{-100.0,-100.0},{100.0,100.0}}),
      graphics={Text(lineColor={0,0,255},extent={{-150,150},{150,110}},textString = "%name")}),
      Documentation(revisions="<html>
        <!--COPYRIGHT-->
        </html>"));
end DifferentConditions;
