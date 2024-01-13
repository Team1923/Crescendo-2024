package frc.robot.AutoUtils;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoUtils.AutoInstantiator;

public class AutoChooser {
    public enum AutoMode {

	}

	private SendableChooser<AutoMode> chooser;
	ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
	ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
			.withPosition(4, 0)
			.withSize(2, 1);
	
	public AutoChooser(){
		chooser = new SendableChooser<>();
		auto.add(chooser);
	}

	public Command startMode(SwerveSubsystem swerve, IntakeSubsystem intake){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			
			default:
				return null;//change
		}
	}
}
