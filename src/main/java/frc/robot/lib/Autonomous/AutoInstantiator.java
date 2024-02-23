package frc.robot.lib.Autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutoInstantiator {

	public static AutoInstantiator autoInstantiator;

	public PathPlannerAuto straight4MetersAuto;
	
	public PathPlannerAuto twoSub;
	public PathPlannerAuto threeSub;
	public PathPlannerAuto fourSub;
	public PathPlannerAuto fiveSub;
	public PathPlannerAuto sixSub;

	public PathPlannerAuto fourFar;
	
	
	

	public static synchronized AutoInstantiator getInstance() {
		if (autoInstantiator == null) {
			autoInstantiator = new AutoInstantiator();
		}
		return autoInstantiator;
	}

	public AutoInstantiator() {
		

		straight4MetersAuto = new PathPlannerAuto("Auto4Meters");

		twoSub = new PathPlannerAuto("2GPSub");
		threeSub = new PathPlannerAuto("3GPSub");
		fourSub = new PathPlannerAuto("4GPSub");
		fiveSub = new PathPlannerAuto("5GPSub");
		sixSub = new PathPlannerAuto("6GPSub");

		fourFar = new PathPlannerAuto("4GPFar");

	}

	public PathPlannerAuto getStraight4MetersAuto(){
		return straight4MetersAuto;
	}

	public PathPlannerAuto getTwoSub() {
		return twoSub;
	}

	public PathPlannerAuto getThreeSub() {
		return threeSub;
	}

	public PathPlannerAuto getFourSub() {
		return fourSub;
	}

	public PathPlannerAuto getFiveSub() {
		return fiveSub;
	}

	public PathPlannerAuto getSixSub() {
		return sixSub;
	}

	public PathPlannerAuto getFourFar() {
		return fourFar;
	}


}
