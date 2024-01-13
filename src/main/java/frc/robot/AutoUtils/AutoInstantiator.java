package frc.robot.AutoUtils;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoInstantiator {

	public static AutoInstantiator autoInstantiator;
	private PathPlannerAuto testAuto;
	
    //Auto from pathplanner, not new sequential command
	public static synchronized AutoInstantiator getInstance() {
		if (autoInstantiator == null) {
			autoInstantiator = new AutoInstantiator();
		}
		return autoInstantiator;
	}

	public AutoInstantiator() {
		testAuto = new PathPlannerAuto("Test_Auto");
	}

	public PathPlannerAuto getTestAuto() {
		return testAuto;
	}

}
