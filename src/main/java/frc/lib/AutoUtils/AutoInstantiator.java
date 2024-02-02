package frc.lib.AutoUtils;


import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutoInstantiator {

	public static AutoInstantiator autoInstantiator;

    public PathPlannerAuto basicAuto;
	public PathPlannerAuto straight4MetersAuto;
	
	public static synchronized AutoInstantiator getInstance() {
		if (autoInstantiator == null) {
			autoInstantiator = new AutoInstantiator();
		}
		return autoInstantiator;
	}

	public AutoInstantiator() {
		basicAuto = new PathPlannerAuto("BasicAuto");
		straight4MetersAuto = new PathPlannerAuto("Auto4Meters");

	}

	public PathPlannerAuto getBasicAuto() {
		return basicAuto;
	}

	public PathPlannerAuto getstraight4MetersAuto(){
		return straight4MetersAuto;
	}


}