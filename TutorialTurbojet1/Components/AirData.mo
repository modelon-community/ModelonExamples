within TutorialTurbojet1.Components;
model AirData
    extends .Modelon.Environment.AirDataImplementations.XYZ(Mn_par = 0,alt_par = 0);
    
    annotation (
        defaultComponentName="airData",
        defaultComponentPrefixes="inner");
end AirData;
