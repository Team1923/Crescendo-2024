package frc.robot.lib.Autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser {
    public enum AutoMode {
		STRAIGHT_4_METERS_AUTO,
		FOUR_AMP,
		CENTER4_MLR,
		CENTER4_MRL,
		SOURCE_5_AND_4,
		SOURCE_5_AND_3,
		SOURCE_4_AND_3,
		SOURCE_4_AND_5,
		SOURCE_3_AND_4,
		SOURCE_3_AND_5,
		SOURCE_5_AND_3_SUB;
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

		chooser.addOption("4 Gamepiece from Amp Side", AutoMode.FOUR_AMP);

		/*
		 * Center Line Autos
		 */
		chooser.addOption("Center4 Mid Left Right", AutoMode.CENTER4_MLR);
		chooser.addOption("Center4 Mid Right Left", AutoMode.CENTER4_MRL);

		/*
		 * Source Autos
		 */
		chooser.addOption("3 Source 5 and 4", AutoMode.SOURCE_5_AND_4);
		chooser.addOption("3 Source 5 and 3", AutoMode.SOURCE_5_AND_3);
		chooser.addOption("3 Source 4 and 3", AutoMode.SOURCE_4_AND_3);
		chooser.addOption("3 Source 4 and 5", AutoMode.SOURCE_4_AND_5);
		chooser.addOption("3 Source 3 and 5", AutoMode.SOURCE_3_AND_5);
		chooser.addOption("3 Source 3 and 4", AutoMode.SOURCE_3_AND_4);

		chooser.addOption("3 Source from Sub 5 and 3", AutoMode.SOURCE_5_AND_3_SUB);



		auto.add(chooser);
	}

	public Command startMode(){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case FOUR_AMP:
				return autoInstantiator.getFourAmpSide();
			case STRAIGHT_4_METERS_AUTO:
				return autoInstantiator.getStraight4MetersAuto();
			case CENTER4_MLR:
				return autoInstantiator.getFourCenterMLR();
			case CENTER4_MRL:
				return autoInstantiator.getFourCenterMRL();
			case SOURCE_5_AND_4:
				return autoInstantiator.getThreeSource5and4();
			case SOURCE_5_AND_3:
				return autoInstantiator.getThreeSource5and3();
			case SOURCE_3_AND_4:
				return autoInstantiator.getThreeSource3and4();
			case SOURCE_3_AND_5:
				return autoInstantiator.getThreeSource3and5();
			case SOURCE_4_AND_3:
				return autoInstantiator.getThreeSource4and3();
			case SOURCE_4_AND_5:
				return autoInstantiator.getThreeSource4and5();
			case SOURCE_5_AND_3_SUB:
				return autoInstantiator.getThreeSourceSub5and3();
			default:
				return null;
		}
	}
}
