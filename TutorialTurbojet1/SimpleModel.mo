        within TutorialTurbojet1;
model SimpleModel "Model built in the first tutorial section"

    inner .JetPropulsion.Settings_JPL settings_JPL(usePbS = true) annotation(Placement(transformation(extent = {{-40.7625,-100.0},{-20.762500000000003,-80.0}},origin = {0.0,0.0},rotation = 0.0)));

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
        .JetPropulsion.Basic.Inlets.RamPressureRecovery.MachNumberDependent)
      annotation(Placement(transformation(extent = {{-52.05,0.0},{-32.05,20.0}},origin = {0.0,0.0},rotation = 0.0)));

    .JetPropulsion.Basic.Compressors.Compressor cmp(
      nPorts = 1,
      prDes = 7,
      effPolyPrscr = true,
      redeclare replaceable model RlineMap = .JetPropulsion.Basic.Compressors.RlineMaps.Default (
        redeclare replaceable package Table =
            .JetPropulsion.Compressors.Sections.Examples.JT9D.Components.Maps.HighPressure))
        annotation(Placement(transformation(extent = {{-20.05,0.0},{-0.05000000000000071,20.0}},origin = {0.0,0.0},rotation = 0.0)));

    .JetPropulsion.Basic.Burners.WithInputPort brn(
      switchBurn = .JetPropulsion.Basic.Burners.Internals.SwitchBurn.Temperature,
      TtCombOutPrscrPar = 1166.483,
      useInput = false,
      redeclare replaceable model eff =
        .JetPropulsion.Basic.Burners.Correlations.Efficiency.Constant (eff = 0.98),dpqpf = 0.05)
        annotation(Placement(transformation(extent = {{21.95,0.0},{41.95,20.0}},origin = {0.0,0.0},rotation = 0.0)));

    .JetPropulsion.Basic.Turbines.Turbine trb(
      nPorts = 1,
      NDesPrscr = 16500,
      effPolyPrscr = true,
      effPolyDesPrscr = 0.83,
      redeclare replaceable model PrMap = .JetPropulsion.Basic.Turbines.PRmaps.Default (
        redeclare replaceable package Table =
            .JetPropulsion.Turbines.Sections.Examples.JT9D.Components.Maps.HighPressure))
        annotation(Placement(transformation(extent = {{61.95,0.0},{81.95,20.0}},origin = {0.0,0.0},rotation = 0.0)));

    .JetPropulsion.Basic.Nozzles.Nozzle noz(
      redeclare replaceable model Friction =
        .JetPropulsion.Basic.Nozzles.Correlations.Friction.Constant (dpqpPrscr = 0.1),
      useCV = false,
      redeclare replaceable model ConDivThrustCoefficient =
        .JetPropulsion.Basic.Nozzles.Correlations.ThrustCoefficient.ConDiv.Constant(CFgPrscr = 0.98))
        annotation(Placement(transformation(extent = {{101.95,0.0},{121.95,20.0}},origin = {0.0,0.0},rotation = 0.0)));

  // Aggregate Properties
  //Real NetThrust_Fg_US=Modelon.Units.Conversions.to_lbf(cycleProperties.Fn);
  //Real NetFuelFlowRate_Wfuel_US=Modelon.Units.Conversions.to_lbmPerS(cycleProperties.wFuel);
  //Real NetSFC_SFC_US=cycleProperties.SFC;
    
    
equation
  connect(ambient.port, inlet.portA)
    annotation (Line(points={{-57.25,10},{-46.05,10}}, color={255,128,0}));
    connect(inlet.portB,cmp.portA) annotation(Line(points = {{-37.25,10},{-20.05,10}},color = {255,128,0}));
    connect(cmp.portB[1],brn.portA) annotation(Line(points = {{0.75,10},{21.95,10}},color = {255,128,0}));
    connect(brn.portB,trb.portA) annotation(Line(points = {{42.75,10},{61.95,10}},color = {255,128,0}));
    connect(trb.portB[1],noz.port) annotation(Line(points = {{82.75,10},{101.95,10}},color = {255,128,0}));
    connect(cmp.flange,trb.flange) annotation(Line(points = {{-10.05,0},{-10.05,-16},{71.95,-16},{71.95,0}},color = {0,0,0}));

    annotation (
      Diagram(coordinateSystem(extent = {{-120.0,-100.0},{160.0,100.0}})),
      Icon(coordinateSystem(preserveAspectRatio = false,
      extent = {{-100.0,-100.0},{100.0,100.0}}),
      graphics={Text(lineColor={0,0,255},extent={{-150,150},{150,110}},textString = "%name")}),
      Documentation(revisions="<html>
        <!--COPYRIGHT-->
        </html>"));
end SimpleModel;
