within TutorialTurbojet1.Components;
model Aggregate
    extends .JetPropulsion.CyclePerformance.Aggregates.Turbojet;
    annotation(
        defaultComponentName="cycleProperties",
        defaultComponentPrefixes="inner");
end Aggregate;
