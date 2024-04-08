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

	public PathPlannerAuto fourAmpTrap;

	/*
	 * Different 4 Center Note Autos
	 */

	 public PathPlannerAuto fourCenterLMR;
	 public PathPlannerAuto fourCenterLRM;
	 public PathPlannerAuto fourCenterMRL;
	 public PathPlannerAuto fourCenterMLR;
	 public PathPlannerAuto fourCenterRLM;
	 public PathPlannerAuto fourCenterRML;



	public static synchronized AutoInstantiator getInstance() {
		if (autoInstantiator == null) {
			autoInstantiator = new AutoInstantiator();
		}
		return autoInstantiator;
	}

	public AutoInstantiator() {
		

		straight4MetersAuto = new PathPlannerAuto("Auto4Meters");

		fiveSub = new PathPlannerAuto("5GPSub");


		sixOptimized = new PathPlannerAuto("6GPSubOptimized");


		fourSource = new PathPlannerAuto("4SourceSide");

		fourAmpSide = new PathPlannerAuto("4AmpSide");
		midLineShove = new PathPlannerAuto("MidLineShove");
		fourAmpTrap = new PathPlannerAuto("AmpTrap");

		/*
		 * Center Autos
		 */
	 fourCenterLMR = new PathPlannerAuto("LMR");
	 fourCenterLRM = new PathPlannerAuto("LRM");
	 fourCenterMRL = new PathPlannerAuto("MRL");
	 fourCenterMLR = new PathPlannerAuto("MLR");
	 fourCenterRLM = new PathPlannerAuto("RLM");
	 fourCenterRML = new PathPlannerAuto("RML");

	}

	public PathPlannerAuto getFourCenterLMR() {
		return fourCenterLMR;
	}

	public PathPlannerAuto getFourCenterLRM() {
		return fourCenterLRM;
	}

	public PathPlannerAuto getFourCenterMRL() {
		return fourCenterMRL;
	}

	public PathPlannerAuto getFourCenterMLR() {
		return fourCenterMLR;
	}

	public PathPlannerAuto getFourCenterRLM() {
		return fourCenterRLM;
	}

	public PathPlannerAuto getFourCenterRML() {
		return fourCenterRML;
	}

	public PathPlannerAuto getFourAmpTrap() {
		return fourAmpTrap;
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
