within Tutorial.Experiment;
model TankTestBase
    .Modelon.ThermoFluid.Volumes.TwoPhaseTankSimpleWall storageTank(enable_externalHeatPort = true,N_vaporPort = 1,N_liquidPort = 1,redeclare replaceable package Medium = .VaporCycle.Media.Naturals.HydrogenMbwr,redeclare replaceable .Modelon.ThermoFluid.Volumes.SubComponents.TankShapes.HorizontalCylinderEllipsoid geo(r_internal = r_inner,s_end = r_inner,L = L_cylinder,t_wall = t_insulation),redeclare replaceable .Modelon.ThermoFluid.Volumes.HeatTransfer.ConstantCoefficient internalConvection(alpha_dry_prescribed = fill(alpha_wall,2),alpha_wet_prescribed = fill(alpha_wall,2)),K_cond_surface = 1e-3,g_external = k_insulation / t_insulation,k_wall = k_insulation,p_start = p_start,initFromLevel = false,V_liq_start = V_liq_start,vaporLiquidHeatTransfer(alpha = 4)) annotation(Placement(transformation(extent = {{-30.0,6.0},{-10.0,26.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelon.ThermoFluid.Valves.CheckValve checkValve(zeta_A = 0.1,A_open = 1e-4,dp_open = p_max - 101305,dp_closed = checkValve.dp_open - 10000,relativeOpening_start = 0,redeclare replaceable package Medium = .VaporCycle.Media.Naturals.HydrogenMbwr) annotation(Placement(transformation(extent = {{-8.0,29.0},{12.0,49.0}},origin = {0.0,0.0},rotation = 0.0)));
    .VaporCycle.Sources.TwoPhasePressureSource Atmosphere(N = 1,p = pamb,energyInput = Modelon.ThermoFluid.Choices.EnergyDefinition.T,T = Tamb,redeclare replaceable package Medium = .VaporCycle.Media.Naturals.HydrogenMbwr) annotation(Placement(transformation(extent = {{46.0,52.0},{26.0,32.0}},origin = {0.0,0.0},rotation = 0.0)));
    .VaporCycle.Sources.TwoPhaseFlowSource massFlowBoundary(redeclare replaceable package Medium = .VaporCycle.Media.Naturals.HydrogenMbwr,use_m_flow_in = true,energyInput = Modelon.ThermoFluid.Choices.EnergyDefinition.T,T = 20) annotation(Placement(transformation(extent = {{-48.0,-36.0},{-28.0,-16.0}},origin = {0.0,0.0},rotation = 0.0)));
    .Modelon.ThermoFluid.Sources.Environment_T ambient_T(T0 = Tamb) annotation(Placement(transformation(extent = {{-72.0,-10.0},{-52.0,10.0}},origin = {0.0,0.0},rotation = 0.0)));
    inner .Modelon.Environment.AtmosphereImplementations.ISA atmosphere annotation(Placement(transformation(extent = {{-67.0,48.0},{-23.0,68.0}},origin = {0.0,0.0},rotation = 0.0)));
    inner .Modelon.Environment.AirDataImplementations.XYZ airData(alt_input = true,use_alt = true) annotation(Placement(transformation(extent = {{-104.0,48.0},{-84.0,68.0}},origin = {0.0,0.0},rotation = 0.0)));
    parameter .Modelica.Units.SI.AbsolutePressure p_start = 101325 "Initial pressure in the tank" annotation(Dialog(group = "Initialization"));
    parameter .Modelica.Units.SI.Mass massHydAvail(fixed = true) "Mass of hydrogen in the tank" annotation(Dialog(group = "Initialization"));
    parameter .Modelica.Units.SI.AbsolutePressure p_max "Maximum tank pressure" annotation(Dialog(group = "Tank design"));
    parameter .Modelica.Units.SI.Volume V_inner "Internal volume of the Tank" annotation(Dialog(group = "Tank geometry"));
    parameter .Modelica.Units.SI.Length D_outer "Outer diameter of the Tank" annotation(Dialog(group = "Tank geometry"));
    parameter .Modelica.Units.SI.Length L_total "Total length of the Tank" annotation(Dialog(group = "Tank geometry"));
    parameter .Modelica.Units.SI.Length t_vessel "Vessel layer thickness (inner most)" annotation(Dialog(group = "Wall parameters"));
    parameter .Modelica.Units.SI.Length t_insulation "Insulation layer thickness (middle)" annotation(Dialog(group = "Wall parameters"));
    parameter .Modelica.Units.SI.Length t_protection "Protection layer thickness (outer most)" annotation(Dialog(group = "Wall parameters"));
    parameter .Modelica.Units.SI.ThermalConductivity k_insulation "Insulation thermal conductivity" annotation(Dialog(group = "Wall parameters"));
    parameter .Modelica.Units.SI.CoefficientOfHeatTransfer alpha_wall = 100 "Heat transfer coefficient of the wall" annotation(Dialog(group = "Wall parameters"));
    parameter .Modelica.Units.SI.Length r_outer = D_outer / 2 "Outer radius of the tank" annotation(Dialog(group = "Final parameters"));
    parameter .Modelica.Units.SI.Length r_inner = r_outer - t_wall "Inner radius of the tank" annotation(Dialog(group = "Final parameters"));
    parameter .Modelica.Units.SI.Length D_inner = 2 * r_inner "Internal diameter of the Tank" annotation(Dialog(group = "Final parameter"));
    parameter .Modelica.Units.SI.Length L_cylinder = L_total - D_outer "Length of the cylindrical section between the hemispherical end caps" annotation(Dialog(group = "Final parameter"));
    .Modelica.Units.SI.Temperature Tamb = atmosphere.temperature(r=airData.r) "Ambient temperature";
    .Modelica.Units.SI.AbsolutePressure pamb (start = 101325)= atmosphere.pressure(r=airData.r) "Ambient pressure";  
    parameter .Modelica.Units.SI.Length t_wall = t_vessel + t_insulation + t_protection "Total wall thickness" annotation(Dialog(group = "Final parameters"));
protected
    parameter .Modelica.Units.SI.Density d_liq_start = storageTank.Medium.density_phX(p_start,storageTank.h_liq_start) "Ambient Temperature";
    parameter .Modelica.Units.SI.Density d_vap_start = storageTank.Medium.density_phX(p_start,storageTank.h_vap_start) "Ambient Pressure";
    parameter .Modelica.Units.SI.Volume V_liq_start(fixed = false) "Initial liquid volume tank";
    parameter .Modelica.Units.SI.Volume V_vap_start(fixed = false) "Initial vapour volume in tank";
initial equation
    //Find the starting liquid volume from the total mass of hydrogen available
    massHydAvail = V_liq_start*d_liq_start + V_vap_start*d_vap_start "Mass balance";
    V_inner = V_liq_start + V_vap_start;
equation
    connect(checkValve.portA,storageTank.vaporPort[1]) annotation(Line(points = {{-8,39},{-20,39},{-20,25}},color = {255,128,0}));
    connect(checkValve.portB,Atmosphere.port[1]) annotation(Line(points = {{12,39},{21,39},{21,42},{28,42}},color = {255,128,0}));
    connect(storageTank.liquidPort[1],massFlowBoundary.port) annotation(Line(points = {{-20,7},{-20,-26},{-30.2,-26}},color = {0,190,0}));
    connect(storageTank.heat_external,ambient_T.port[1]) annotation(Line(points = {{-28.7,16.1},{-62,16.1},{-62,10}},color = {191,0,0}));
    annotation(Icon(coordinateSystem(preserveAspectRatio = false,extent = {{-100.0,-100.0},{100.0,100.0}}),graphics = {Rectangle(lineColor={0,0,0},fillColor={230,230,230},fillPattern=FillPattern.Solid,extent={{-100.0,-100.0},{100.0,100.0}}),Text(lineColor={0,0,255},extent={{-150,150},{150,110}},textString="%name")}),
    Documentation(info = "<html><p>This is an example model for the <a href=\"https://help.modelon.com/latest/tutorials/hydrogen_storage/refueling/\">tank refueling tutorial</a>, section <a href=\"https://help.modelon.com/latest/tutorials/hydrogen_storage/refueling/#base-class-setup-to-test\">Base Class setup to test</a>.</p></html>"));
end TankTestBase;
