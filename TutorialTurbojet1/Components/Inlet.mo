within TutorialTurbojet1.Components;
model Inlet "Inlet parameterized for notional J85"
    extends .JetPropulsion.Basic.Inlets.Inlet(
      redeclare replaceable model RPR =
        .JetPropulsion.Basic.Inlets.RamPressureRecovery.MachNumberDependent);
    annotation(defaultComponentName = "inl");
end Inlet;
