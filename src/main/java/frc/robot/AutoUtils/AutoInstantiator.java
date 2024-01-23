package frc.robot.AutoUtils;

import java.nio.file.Path;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoInstantiator {

	public static AutoInstantiator autoInstantiator;
	private PathPlannerAuto testAuto;
	private PathPlannerAuto testStraight;
	private PathPlannerAuto testRotate;

	
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
		testRotate = new PathPlannerAuto("Test_Rotate");
	}

	public PathPlannerAuto getTestAuto() {
		return testAuto;
	}

	public PathPlannerAuto getTestStraight(){
		return testStraight;
	}

	public PathPlannerAuto getTestRotate(){
		return testRotate;
	}


}
