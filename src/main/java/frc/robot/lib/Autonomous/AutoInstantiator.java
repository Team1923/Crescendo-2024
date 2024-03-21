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

	public PathPlannerAuto sixOptimized;


	public PathPlannerAuto fiveMid; 
	public PathPlannerAuto threeMid;	

	// public PathPlannerAuto fourFar;
	public PathPlannerAuto fourSource;

	public PathPlannerAuto twoTest;
	public PathPlannerAuto fourTest;

	public PathPlannerAuto fourAmpSide;
	public PathPlannerAuto midLineShove;

	public static synchronized AutoInstantiator getInstance() {
		if (autoInstantiator == null) {
			autoInstantiator = new AutoInstantiator();
		}
		return autoInstantiator;
	}

	public AutoInstantiator() {
		

		straight4MetersAuto = new PathPlannerAuto("Auto4Meters");

		// twoSub = new PathPlannerAuto("2GPSub");
		// threeSub = new PathPlannerAuto("3GPSub");
		// fourSub = new PathPlannerAuto("4GPSub");
		fiveSub = new PathPlannerAuto("5GPSub");
		// sixSub = new PathPlannerAuto("6GPSub");

		sixOptimized = new PathPlannerAuto("6GPSubOptimized");
		

		// fiveMid = new PathPlannerAuto("5GPMid");

		// fourFar = new PathPlannerAuto("4GPFar");
		
		// threeMid = new PathPlannerAuto("3GPMid");

		fourSource = new PathPlannerAuto("4SourceSide");

		fourAmpSide = new PathPlannerAuto("4AmpSide");
		midLineShove = new PathPlannerAuto("MidLineShove");

	}

	public PathPlannerAuto getFourAmpSide() {
		return fourAmpSide;
	}

	public PathPlannerAuto getMidLineShove() {
		return midLineShove;
	}

	public PathPlannerAuto getStraight4MetersAuto(){
		return straight4MetersAuto;
	}

	public PathPlannerAuto get6Optimized(){
		return sixOptimized;
	}
	

	// public PathPlannerAuto getTwoSub() {
	// 	return twoSub;
	// }

	// public PathPlannerAuto getThreeSub() {
	// 	return threeSub;
	// }

	// public PathPlannerAuto getFourSub() {
	// 	return fourSub;
	// }

	public PathPlannerAuto getFiveSub() {
		return fiveSub;
	}

	// public PathPlannerAuto getSixSub() {
	// 	return sixSub;
	// }



	// public PathPlannerAuto getFiveMid(){
	// 	return fiveMid;
	// }

	public PathPlannerAuto getFourSource(){
		return fourSource;
	}



	// public PathPlannerAuto getFourFar() {
	// 	return fourFar;
	// }



	// public PathPlannerAuto getMidThree(){
	// 	return threeMid;
	// }



}
