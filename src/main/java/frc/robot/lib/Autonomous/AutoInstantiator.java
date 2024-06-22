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

	public PathPlannerAuto subTaxi;

	/*
	 * Different 4 Center Note Autos
	 */

	 public PathPlannerAuto fourCenterLMR;
	 public PathPlannerAuto fourCenterLRM;
	 public PathPlannerAuto fourCenterMRL;
	 public PathPlannerAuto fourCenterMLR;
	 public PathPlannerAuto fourCenterRLM;
	 public PathPlannerAuto fourCenterRML;

	 /*
	  * Different 3 Note Source Side
	  */

	  public PathPlannerAuto threeSource5and4;
	  public PathPlannerAuto threeSource5and3;
	  public PathPlannerAuto threeSource4and3;
	  public PathPlannerAuto threeSource4and5;
	  public PathPlannerAuto threeSource3and4;
	  public PathPlannerAuto threeSource3and5;

	  /*
	   * Different 3 Note Source Side from Sub
	   */
	  public PathPlannerAuto threeSourceSub5and3;
	  public PathPlannerAuto threeSourceSub5and4;
	  public PathPlannerAuto threeSourceSub4and3;
	  public PathPlannerAuto threeSourceSub4and5;
	  public PathPlannerAuto threeSourceSub3and4;
	  public PathPlannerAuto threeSourceSub3and5;







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
		fourAmpTrap = new PathPlannerAuto("AmpTrap");

		midLineShove = new PathPlannerAuto("MidLineShove");

		subTaxi = new PathPlannerAuto("SubTaxiAuto");


		/*
		 * Center Autos
		 */
	 fourCenterLMR = new PathPlannerAuto("LMR");
	 fourCenterLRM = new PathPlannerAuto("LRM");
	 fourCenterMRL = new PathPlannerAuto("MRL");
	 fourCenterMLR = new PathPlannerAuto("MLR");
	 fourCenterRLM = new PathPlannerAuto("RLM");
	 fourCenterRML = new PathPlannerAuto("RML");

		/*
		 * Source Autos
		 */
	threeSource5and4 = new PathPlannerAuto("SourceSide5and4");
	threeSource5and3 = new PathPlannerAuto("SourceSide5and3");
	threeSource4and3 = new PathPlannerAuto("SourceSide4and3");
	threeSource4and5 = new PathPlannerAuto("SourceSide4and5");
	threeSource3and5 = new PathPlannerAuto("SourceSide3and5");
	threeSource3and4 = new PathPlannerAuto("SourceSide3and4");

	/*
	 * Source Side Sub Autos
	 */
	threeSourceSub5and3 = new PathPlannerAuto("SourceSideSub5and3");
	threeSourceSub5and4 = new PathPlannerAuto("SourceSideSub5and4");
	threeSourceSub4and3 = new PathPlannerAuto("SourceSideSub4and3");
	threeSourceSub4and5 = new PathPlannerAuto("SourceSideSub4and5");
	threeSourceSub3and4 = new PathPlannerAuto("SourceSideSub3and4");
	threeSourceSub3and5 = new PathPlannerAuto("SourceSideSub3and5");
		



	}

	public PathPlannerAuto getThreeSourceSub5and4() {
		return threeSourceSub5and4;
	}

	public PathPlannerAuto getThreeSourceSub4and3() {
		return threeSourceSub4and3;
	}

	public PathPlannerAuto getThreeSourceSub4and5() {
		return threeSourceSub4and5;
	}

	public PathPlannerAuto getThreeSourceSub3and4() {
		return threeSourceSub3and4;
	}

	public PathPlannerAuto getThreeSourceSub3and5() {
		return threeSourceSub3and5;
	}

	public PathPlannerAuto getThreeSourceSub5and3() {
		return threeSourceSub5and3;
	}

	public PathPlannerAuto getThreeSource5and4() {
		return threeSource5and4;
	}

	public PathPlannerAuto getThreeSource5and3() {
		return threeSource5and3;
	}

	public PathPlannerAuto getThreeSource4and3() {
		return threeSource4and3;
	}

	public PathPlannerAuto getThreeSource4and5() {
		return threeSource4and5;
	}

	public PathPlannerAuto getThreeSource3and4() {
		return threeSource3and4;
	}

	public PathPlannerAuto getThreeSource3and5() {
		return threeSource3and5;
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

	public PathPlannerAuto getSubTaxi(){
		return subTaxi;
	}






}
