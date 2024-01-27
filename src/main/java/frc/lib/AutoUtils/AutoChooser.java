package frc.lib.AutoUtils;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AutoUtils.AutoInstantiator;

public class AutoChooser {
    public enum AutoMode {
		BASIC_AUTO,
		STRAIGHT_4_METERS_AUTO,
		FOUR_NOTE_AUTO
	}

	private SendableChooser<AutoMode> chooser;
	ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
	ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
			.withPosition(4, 0)
			.withSize(2, 1);
	
	private AutoInstantiator autoInstantiator = AutoInstantiator.getInstance();

	public AutoChooser(){
		chooser = new SendableChooser<>();
		chooser.addOption("Basic Auto", AutoMode.BASIC_AUTO);
		chooser.addOption("Straight 4 Meter Auto", AutoMode.STRAIGHT_4_METERS_AUTO);
		chooser.addOption("4 Note Auto", AutoMode.FOUR_NOTE_AUTO);
		auto.add(chooser);
	}

	public Command startMode(){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case BASIC_AUTO:
				return autoInstantiator.getBasicAuto();
			case STRAIGHT_4_METERS_AUTO:
				return autoInstantiator.getstraight4MetersAuto();
			case FOUR_NOTE_AUTO:
				return autoInstantiator.get4NotesAuto();
			default:
				return null;
		}
	}
}