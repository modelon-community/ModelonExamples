within TutorialTurbojet1.Components;
model ControlBurner
    extends .JetPropulsion.CyclePerformance.Control.Burner.FarFromOutletTempAndSpeed(
       TtCombOutPrscrPar = 1166,
       fix = false,
       useConstraint = true,
       useConstraintOnly = false);
    annotation(defaultComponentName = "farFromT4AndNc");
end ControlBurner;
