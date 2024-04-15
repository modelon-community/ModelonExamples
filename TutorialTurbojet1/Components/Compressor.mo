within TutorialTurbojet1.Components;
model Compressor "Compressor parameterized for notional J85"
    extends JetPropulsion.Basic.Compressors.Compressor(
      prDes = 7,
      effPolyPrscr = true,
      redeclare replaceable model RlineMap = .JetPropulsion.Basic.Compressors.RlineMaps.Default (
        redeclare replaceable package Table =
            .JetPropulsion.Compressors.Sections.Examples.JT9D.Components.Maps.HighPressure));
    annotation(defaultComponentName = "cmp");
end Compressor;
