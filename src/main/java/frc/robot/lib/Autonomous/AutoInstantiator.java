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

	public PathPlannerAuto fiveSubNoGoalCentric;
	public PathPlannerAuto fiveSubNoAutoCommands;

	public PathPlannerAuto fiveMid; 
	public PathPlannerAuto threeMid;	

	// public PathPlannerAuto fourFar;
	public PathPlannerAuto fourSource;

	public PathPlannerAuto twoTest;
	public PathPlannerAuto fourTest;

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

		fiveSubNoAutoCommands = new PathPlannerAuto("5GPSubNonAuto");
		fiveSubNoGoalCentric = new PathPlannerAuto("5GPSubNoGoalCentric");

		fiveMid = new PathPlannerAuto("5GPMid");

		// fourFar = new PathPlannerAuto("4GPFar");
		
		twoTest = new PathPlannerAuto("2Test");
		fourTest = new PathPlannerAuto("4Test");
		threeMid = new PathPlannerAuto("3GPMid");

		fourSource = new PathPlannerAuto("4SourceSide");

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

	public PathPlannerAuto getFiveSubNoAutoCommands(){
		return fiveSubNoAutoCommands;
	}

	public PathPlannerAuto getFiveSubNoGoalCentric(){
		return fiveSubNoGoalCentric;
	}


	public PathPlannerAuto getFiveMid(){
		return fiveMid;
	}

	public PathPlannerAuto getFourSource(){
		return fourSource;
	}



	// public PathPlannerAuto getFourFar() {
	// 	return fourFar;
	// }


	public PathPlannerAuto getTwoTest() {
		return twoTest;
	}

	public PathPlannerAuto getFourTest() {
		return fourTest;
	}

	public PathPlannerAuto getMidThree(){
		return threeMid;
	}



}
