package frc.robot.lib.Autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser {
    public enum AutoMode {
		STRAIGHT_4_METERS_AUTO,
		SUBWOOFER_2_GP,
		SUBWOOFER_3_GP,
		SUBWOOFER_4_GP,
		SUBWOOFER_5_GP,
		SUBWOOFER_6_GP,
		MID_5_GP,
		FOUR_SOURCE,
		// FAR_4_GP,
		MID_THREE;
	}

	private SendableChooser<AutoMode> chooser;
	
	ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
	ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
			.withPosition(3, 0)
			.withSize(2, 1);
	
	private AutoInstantiator autoInstantiator = AutoInstantiator.getInstance();

	public AutoChooser(){
		chooser = new SendableChooser<>();
		chooser.addOption("Straight 4 Meter Auto", AutoMode.STRAIGHT_4_METERS_AUTO);

		chooser.addOption("2 Gamepiece from Subwoofer", AutoMode.SUBWOOFER_2_GP);
		chooser.addOption("3 Gamepiece from Subwoofer", AutoMode.SUBWOOFER_3_GP);
		chooser.addOption("4 Gamepiece from Subwoofer", AutoMode.SUBWOOFER_4_GP);
		chooser.addOption("5 Gamepiece from Subwoofer", AutoMode.SUBWOOFER_5_GP);
		chooser.addOption("6 Gamepiece from Subwoofer", AutoMode.SUBWOOFER_6_GP);

		

		chooser.addOption("5 Gamepiece Mid Rush", AutoMode.MID_5_GP);
		
		// chooser.addOption("4 Gamepiece from Far Side", AutoMode.FAR_4_GP);


		chooser.addOption("Mid 3", AutoMode.MID_THREE);

		chooser.addOption("Four Souce", AutoMode.FOUR_SOURCE);

		auto.add(chooser);
	}

	public Command startMode(){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case SUBWOOFER_2_GP:
				return autoInstantiator.getTwoSub();
			case SUBWOOFER_3_GP:
				return autoInstantiator.getThreeSub();
			case SUBWOOFER_4_GP:
				return autoInstantiator.getFourSub();
			case SUBWOOFER_5_GP:
				return autoInstantiator.getFiveSub();
			case SUBWOOFER_6_GP:
				return autoInstantiator.getSixSub();
	
			// case FAR_4_GP:
			// 	return autoInstantiator.getFourFar();
			
			case STRAIGHT_4_METERS_AUTO:
				return autoInstantiator.getStraight4MetersAuto();

			case MID_5_GP:
				return autoInstantiator.getFiveMid();
			case MID_THREE:
				return autoInstantiator.getMidThree();
			case FOUR_SOURCE:
				return autoInstantiator.getFourSource();

			default:
				return null;
		}
	}
}
