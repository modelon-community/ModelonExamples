within TutorialTurbojet1.Components;
model Burner "Burner parameterized for notional J85"
    extends .JetPropulsion.Basic.Burners.WithInputPort(
      switchBurn = .JetPropulsion.Basic.Burners.Internals.SwitchBurn.Temperature,
      TtCombOutPrscrPar = 1166.483,
      useInput = false,
      redeclare replaceable model eff =
        .JetPropulsion.Basic.Burners.Correlations.Efficiency.Constant (eff = 0.98),dpqpf = 0.05);
    annotation(defaultComponentName = "brn");
end Burner;
