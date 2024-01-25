package frc.lib.AutoUtils;

import java.nio.file.Path;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoInstantiator {

	public static AutoInstantiator autoInstantiator;

    public PathPlannerAuto basicAuto;
	
    //Auto from pathplanner, not new sequential command
	public static synchronized AutoInstantiator getInstance() {
		if (autoInstantiator == null) {
			autoInstantiator = new AutoInstantiator();
		}
		return autoInstantiator;
	}

	public AutoInstantiator() {
		basicAuto = new PathPlannerAuto("BasicAuto");
	}

	public PathPlannerAuto getBasicAuto() {
		return basicAuto;
	}




}