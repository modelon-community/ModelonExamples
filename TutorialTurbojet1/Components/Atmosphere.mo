within TutorialTurbojet1.Components;
model Atmosphere
    extends .Modelon.Environment.AtmosphereImplementations.US76;
    
    annotation (
        defaultComponentName="atmosphere",
        defaultComponentPrefixes="inner");
end Atmosphere;
