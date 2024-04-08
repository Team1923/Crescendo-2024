package frc.robot.lib.Autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser {
    public enum AutoMode {
		STRAIGHT_4_METERS_AUTO,
		SUBWOOFER_5_GP,
		FOUR_SOURCE,
		FOUR_AMP,
		MID_LINE_SHOVE,
		FOUR_AMP_TRAP,
		SUB_6_OPTIMIZED,
		CENTER4_LMR,
		CENTER4_LRM,
		CENTER4_MLR,
		CENTER4_MRL,
		CENTER4_RLM,
		CENTER4_RML;
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

		chooser.addOption("5 Gamepiece from Subwoofer", AutoMode.SUBWOOFER_5_GP);

		chooser.addOption("4 Gamepiece from Amp Side", AutoMode.FOUR_AMP);

		chooser.addOption("MidLine Shove", AutoMode.MID_LINE_SHOVE);

		chooser.addOption("6 Gamepiece from Sub OPTIMIZED", AutoMode.SUB_6_OPTIMIZED);

		chooser.addOption("4 Amp Side TRap", AutoMode.FOUR_AMP_TRAP);
		

		chooser.addOption("3.5 Note from Souce", AutoMode.FOUR_SOURCE);

		/*
		 * Center Line Autos
		 */
		chooser.addOption("Center4 Left Mid Right", AutoMode.CENTER4_LMR);
		chooser.addOption("Center4 Left Right Mid", AutoMode.CENTER4_LRM);
		chooser.addOption("Center4 Mid Left Right", AutoMode.CENTER4_MLR);
		chooser.addOption("Center4 Mid Right Left", AutoMode.CENTER4_MRL);
		chooser.addOption("Center4 Right Mid Left", AutoMode.CENTER4_RML);
		chooser.addOption("Center4 Right Left Mid", AutoMode.CENTER4_RLM);

		auto.add(chooser);
	}

	public Command startMode(){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case SUBWOOFER_5_GP:
				return autoInstantiator.getFiveSub();
			case FOUR_AMP:
				return autoInstantiator.getFourAmpSide();
			case MID_LINE_SHOVE:
				return autoInstantiator.getMidLineShove();
			case FOUR_AMP_TRAP:
				return autoInstantiator.getFourAmpTrap();
			case STRAIGHT_4_METERS_AUTO:
				return autoInstantiator.getStraight4MetersAuto();
			case SUB_6_OPTIMIZED:
				return autoInstantiator.get6Optimized();
			case CENTER4_LMR:
				return autoInstantiator.getFourCenterLMR();
			case CENTER4_LRM:
				return autoInstantiator.getFourCenterLRM();
			case CENTER4_MLR:
				return autoInstantiator.getFourCenterMLR();
			case CENTER4_MRL:
				return autoInstantiator.getFourCenterMRL();
			case CENTER4_RML:
				return autoInstantiator.getFourCenterRML();
			case CENTER4_RLM:
				return autoInstantiator.getFourCenterRLM();
			case FOUR_SOURCE:
				return autoInstantiator.getFourSource();

			default:
				return null;
		}
	}
}
