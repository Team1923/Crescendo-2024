package frc.robot.AutoUtils;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoInstantiator {

	public static AutoInstantiator autoInstantiator;
	private PathPlannerAuto testAuto;
	private PathPlannerAuto testStraight;

	
    //Auto from pathplanner, not new sequential command
	public static synchronized AutoInstantiator getInstance() {
		if (autoInstantiator == null) {
			autoInstantiator = new AutoInstantiator();
		}
		return autoInstantiator;
	}

	public AutoInstantiator() {
		testAuto = new PathPlannerAuto("Test_Auto");
		testStraight = new PathPlannerAuto("Test_Straight");
	}

	public PathPlannerAuto getTestAuto() {
		return testAuto;
	}

	public PathPlannerAuto getTestStraight(){
		return testStraight;
	}


}
