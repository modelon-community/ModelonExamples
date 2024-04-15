within TutorialTurbojet1.Components;
model Turbine "Turbine parameterized for notional J85"
    extends .JetPropulsion.Basic.Turbines.Turbine(
      NDesPrscr = 16500,
      effPolyPrscr = true,
      effPolyDesPrscr = 0.83,
      redeclare replaceable model PrMap = .JetPropulsion.Basic.Turbines.PRmaps.Default (
        redeclare replaceable package Table =
            .JetPropulsion.Turbines.Sections.Examples.JT9D.Components.Maps.HighPressure));
    annotation(defaultComponentName = "trb");
end Turbine;
