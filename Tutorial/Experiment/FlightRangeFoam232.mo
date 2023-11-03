within Tutorial.Experiment;
model FlightRangeFoam232
    extends .Tutorial.Experiment.FlightRangeBase(massHydAvail = 13620,p_max = 404000,V_inner = 235,D_outer = 4.67,L_total = 17.2,t_vessel = 0.0041,t_insulation = 0.1289,t_protection = 0.0008,k_insulation = 0.015,altitudeTable(fileName = Modelica.Utilities.Files.loadResource("modelica://Tutorial/Resources/Foam232/Altitude.txt")),liquidFlowRateTable(fileName = Modelica.Utilities.Files.loadResource("modelica://Tutorial/Resources/Foam232/FuelFlow.txt")),timeSignal(y = time),ramp(duration = 0));
end FlightRangeFoam232;
