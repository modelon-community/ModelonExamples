within TutorialTurbojet1.Components;
model AmbientBoundary "Ambient boundary parameterized for notional J85"
    extends .JetPropulsion.Boundaries.AmbientBoundary(w=19.9581);
    annotation(defaultComponentName = "ambient");
end AmbientBoundary;
