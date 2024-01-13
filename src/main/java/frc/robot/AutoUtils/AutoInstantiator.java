package frc.robot.AutoUtils;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoInstantiator {

	public static AutoInstantiator autoInstantiator;
    private SwerveSubsystem swerve;
	
    //Auto from pathplanner, not new sequential command
	public static synchronized AutoInstantiator getInstance(SwerveSubsystem s) {
		if (autoInstantiator == null) {
			autoInstantiator = new AutoInstantiator(s);
		}
		return autoInstantiator;
	}

	public AutoInstantiator(SwerveSubsystem swerve) {
		this.swerve = swerve;

	}

}
