within TutorialTurbojet1.Components;
model Settings
    extends .JetPropulsion.Settings_JPL(usePbS = true);
    
    annotation(
        defaultComponentName="settings_JPL",
        defaultComponentPrefixes="inner");
end Settings;
