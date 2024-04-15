within TutorialTurbojet1.Components;
model Nozzle "Nozzle parameterized for notional J85"
    extends .JetPropulsion.Basic.Nozzles.Nozzle(
      redeclare replaceable model Friction =
        .JetPropulsion.Basic.Nozzles.Correlations.Friction.Constant (dpqpPrscr = 0.1),
      useCV = false,
      redeclare replaceable model ConDivThrustCoefficient =
        .JetPropulsion.Basic.Nozzles.Correlations.ThrustCoefficient.ConDiv.Constant(CFgPrscr = 0.98));
    annotation(defaultComponentName = "noz");
end Nozzle;
