package frc.robot.AutoUtils;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoUtils.AutoInstantiator;

public class AutoChooser {
    public enum AutoMode {
		TEST_AUTO,
		TEST_STRAIGHT,
		TEST_ROTATE
	}

	private SendableChooser<AutoMode> chooser;
	ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
	ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
			.withPosition(4, 0)
			.withSize(2, 1);
	
	private AutoInstantiator autoInstantiator = AutoInstantiator.getInstance();

	public AutoChooser(){
		chooser = new SendableChooser<>();
		chooser.addOption("Test Auto S-Curve", AutoMode.TEST_AUTO);
		chooser.addOption("Test Straight Path", AutoMode.TEST_STRAIGHT);
		chooser.addOption("Test Rotate Path", AutoMode.TEST_ROTATE);
		auto.add(chooser);
	}

	public Command startMode(SwerveSubsystem swerve, IntakeSubsystem intake){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case TEST_AUTO:
				return autoInstantiator.getTestAuto();
			case TEST_STRAIGHT:
				return autoInstantiator.getTestStraight();
			case TEST_ROTATE:
				return autoInstantiator.getTestRotate();
			default:
				return null;//change
		}
	}

	public Pose2d getSelectedAutoStartPose() {
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case TEST_AUTO:
				return PathPlannerAuto.getStaringPoseFromAutoFile("Test_Auto");
			case TEST_STRAIGHT:
				return PathPlannerAuto.getStaringPoseFromAutoFile("Test_Straight");
			case TEST_ROTATE:
				return PathPlannerAuto.getStaringPoseFromAutoFile("Test_Rotate");
			default:
				return new Pose2d();
		}
	}
}
