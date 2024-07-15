package frc.robot.lib.Autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoCommand.AutoIntake;
import frc.robot.lib.StateMachine.StateHandler;

public class AutoChooser {

	StateHandler stateHandler = StateHandler.getInstance();

    public enum AutoMode {
		STRAIGHT_4_METERS_AUTO,
		FOUR_AMP,
		FOUR_AMP21,
		MIDLINE_SHOVE,
		SUB_TAXI,
		CENTER4_MLR,
		CENTER4_MRL,
		CENTER4_RML,
		CENTER4_LMR,
		CENTER4_RLM,
		CENTER4_LRM,
		CENTER4_MP,
		CENTER4_PM,
		SOURCE_5_AND_4,
		SOURCE_5_AND_3,
		SOURCE_4_AND_3,
		SOURCE_4_AND_5,
		SOURCE_3_AND_4,
		SOURCE_3_AND_5,
		SOURCE_5_AND_3_SUB,
		SOURCE_5_AND_4_SUB,
		SOURCE_4_AND_3_SUB,
		SOURCE_4_AND_5_SUB,
		SOURCE_3_AND_4_SUB,
		SOURCE_3_AND_5_SUB,
		SOURCE_5_SUB;
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
		chooser.addOption("4 Gamepiece from Amp side Order: 2, 1", AutoMode.FOUR_AMP21);

		/*
		 * Center Line Autos
		 */
		chooser.addOption("Center4 Mid Amp Podium", AutoMode.CENTER4_MLR);
		chooser.addOption("Center4 Mid Podium Amp", AutoMode.CENTER4_MRL);
		chooser.addOption("Center4 Podium Mid Amp", AutoMode.CENTER4_RML);
		chooser.addOption("Center4 Amp Mid Podium", AutoMode.CENTER4_LMR);
		chooser.addOption("Center4 Amp Podium Mid", AutoMode.CENTER4_LRM);
		chooser.addOption("Center4 Podium Amp Mid", AutoMode.CENTER4_RLM);

		chooser.addOption("Center4 Podium Mid (Podium end)", AutoMode.CENTER4_PM);
		chooser.addOption("Center4 Mid Podium (Podium end)", AutoMode.CENTER4_MP);


		chooser.addOption("Mid Line Shove", AutoMode.MIDLINE_SHOVE);

		chooser.addOption("Sub Taxi", AutoMode.SUB_TAXI);





		/*
		 * Source Autos
		 */
		// chooser.addOption("3 Source 5 and 4", AutoMode.SOURCE_5_AND_4);
		// chooser.addOption("3 Source 5 and 3", AutoMode.SOURCE_5_AND_3);
		// chooser.addOption("3 Source 4 and 3", AutoMode.SOURCE_4_AND_3);
		// chooser.addOption("3 Source 4 and 5", AutoMode.SOURCE_4_AND_5);
		// chooser.addOption("3 Source 3 and 5", AutoMode.SOURCE_3_AND_5);
		// chooser.addOption("3 Source 3 and 4", AutoMode.SOURCE_3_AND_4);

		/*
		 * Source Subwoofer Autos
		 */
		chooser.addOption("3 Source from Sub 5 and 3", AutoMode.SOURCE_5_AND_3_SUB);
		chooser.addOption("3 Source from Sub 5 and 4", AutoMode.SOURCE_5_AND_4_SUB);
		chooser.addOption("3 Source from Sub 4 and 3", AutoMode.SOURCE_4_AND_3_SUB);
		chooser.addOption("3 Source from Sub 4 and 5", AutoMode.SOURCE_4_AND_5_SUB);
		chooser.addOption("3 Source from Sub 3 and 4", AutoMode.SOURCE_3_AND_4_SUB);
		chooser.addOption("3 Source from Sub 3 and 5", AutoMode.SOURCE_3_AND_5_SUB);

		chooser.addOption("2 Source from Sub 5 shoot sub", AutoMode.SOURCE_5_SUB);




		auto.add(chooser);
	}

	public Command startMode(){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case FOUR_AMP:
				return autoInstantiator.getFourAmpSide();
			case FOUR_AMP21:
				return autoInstantiator.getFourAmpSide21();
			case STRAIGHT_4_METERS_AUTO:
				return autoInstantiator.getStraight4MetersAuto();
			case SUB_TAXI:
				// stateHandler.setAutoHeadingOffset(-60);
				stateHandler.setAutoHeadingOffset(60);

				return autoInstantiator.getSubTaxi();
			case CENTER4_MLR:
				return autoInstantiator.getFourCenterMLR();
			case CENTER4_MRL:
				return autoInstantiator.getFourCenterMRL();
			case CENTER4_RLM:
				return autoInstantiator.getFourCenterRLM();
			case CENTER4_LRM:
				return autoInstantiator.getFourCenterLRM();
			case CENTER4_LMR:
				return autoInstantiator.getFourCenterLMR();
			case CENTER4_RML:
				return autoInstantiator.getFourCenterRML();
			case CENTER4_MP:
				return autoInstantiator.getFourCenterMidPod();
			case CENTER4_PM:
				return autoInstantiator.getFourCenterPodMid();
			case MIDLINE_SHOVE:
				return autoInstantiator.getMidLineShove();
			// case SOURCE_5_AND_4:
			// 	return autoInstantiator.getThreeSource5and4();
			// case SOURCE_5_AND_3:
			// 	return autoInstantiator.getThreeSource5and3();
			// case SOURCE_3_AND_4:
			// 	return autoInstantiator.getThreeSource3and4();
			// case SOURCE_3_AND_5:
			// 	return autoInstantiator.getThreeSource3and5();
			// case SOURCE_4_AND_3:
			// 	return autoInstantiator.getThreeSource4and3();
			// case SOURCE_4_AND_5:
			// 	return autoInstantiator.getThreeSource4and5();
			case SOURCE_5_AND_3_SUB:
				stateHandler.setAutoHeadingOffset(-60);
				return autoInstantiator.getThreeSourceSub5and3();	
			case SOURCE_5_AND_4_SUB:
				stateHandler.setAutoHeadingOffset(-60);
				return autoInstantiator.getThreeSourceSub5and4();
			case SOURCE_3_AND_4_SUB:
				stateHandler.setAutoHeadingOffset(-60);
				return autoInstantiator.getThreeSourceSub3and4();
			case SOURCE_3_AND_5_SUB:
				stateHandler.setAutoHeadingOffset(-60);
				return autoInstantiator.getThreeSourceSub3and5();
			case SOURCE_4_AND_3_SUB:
				stateHandler.setAutoHeadingOffset(-60);
				return autoInstantiator.getThreeSourceSub4and3();
			case SOURCE_4_AND_5_SUB:
				stateHandler.setAutoHeadingOffset(-60);
				return autoInstantiator.getThreeSourceSub4and5();
			case SOURCE_5_SUB:
				stateHandler.setAutoHeadingOffset(-60);
				return autoInstantiator.getTwoSourceSub5();
			default:
				return null;
		}
	}
}
