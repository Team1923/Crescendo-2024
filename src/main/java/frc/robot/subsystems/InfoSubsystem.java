package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.lib.StateMachine.StateHandler;

public class InfoSubsystem extends SubsystemBase {
  /** Creates a new ShuffleboardSubsystem. */

  public ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
  private StateHandler stateHandler = StateHandler.getInstance();

  private CommandXboxController xboxController;
  private CommandPS4Controller ps4Controller;

  public InfoSubsystem(CommandXboxController x, CommandPS4Controller p){
    this.xboxController = x;
    this.ps4Controller = p;

    if (stateHandler.isPosRPMTuning()){
      GenericEntry POSRPMTUNING = driverDashboard.add("IN POSRPM MODE", false)
      .withSize(3, 3)
      .withPosition(0, 0)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#57F542"))
      .getEntry();

    }
  }

	private GenericEntry ampPos = driverDashboard.add("AMP", false)
			.withSize(3, 1)
			.withPosition(0, 0)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#F59542"))
			.getEntry();

	private GenericEntry subwooferPos = driverDashboard.add("SUBWOOFER", false)
			.withSize(3, 1)
			.withPosition(0, 1)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#CE42F5"))
			.getEntry();
  
  private GenericEntry reverseSubwooferPos = driverDashboard.add("REVERSE_SUBWOOFER", false)
  		.withSize(3, 1)
			.withPosition(0, 2)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#000080"))
			.getEntry();
    private GenericEntry manualClimbing = driverDashboard.add("Manual Climbing", false)
  		.withSize(3, 1)
			.withPosition(2, 1)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#000080"))
			.getEntry();

	private GenericEntry seeSpeakerTag = driverDashboard.add("SPEAKER APRIL TAG", false)
			.withSize(3, 1)
			.withPosition(0, 3)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#57F542"))
			.getEntry();

  private GenericEntry seeTrapTag = driverDashboard.add("TRAP APRIL TAG", false)
  .withSize(3, 1)
  .withPosition(0, 4)
  .withProperties(Map.of("Color when false", "#000000", "Color when true", "#57F542"))
  .getEntry();

  private SuppliedValueWidget<Double> distToSpeaker = driverDashboard.addNumber("DIST TO SPEAKER TAG", () -> stateHandler.getDistanceToSpeakerTag())
          .withPosition(4, 3);

  
  

  @Override
  public void periodic() {
    /* Driver Dashboard Display */
    subwooferPos.setBoolean(stateHandler.getWantToPositionForSubwoofer());
    ampPos.setBoolean(stateHandler.getScoreInAmp());
    reverseSubwooferPos.setBoolean(stateHandler.getReverseSubwoofer());
    seeSpeakerTag.setBoolean(stateHandler.getHasValidSpeakerTag());
    seeTrapTag.setBoolean(stateHandler.getHasValidTrapTag());
    manualClimbing.setBoolean(stateHandler.getManuallyClimbing());

    if(DriverStation.isTeleop() && stateHandler.getBBOneCovered()){
      xboxController.getHID().setRumble(RumbleType.kBothRumble, 0.2);
      ps4Controller.getHID().setRumble(RumbleType.kBothRumble, 0.4);
    } else{
      xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
      ps4Controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    

    /* DEBUG PRINTOUTS - TODO: DISABLE WHEN IN MATCH! */
    /* BEAM BREAK VALUES */
    SmartDashboard.putBoolean("BB ONE COVERED", stateHandler.getBBOneCovered());
    SmartDashboard.putBoolean("BB TWO COVERED", stateHandler.getBBTwoCovered());
    SmartDashboard.putBoolean("BB THREE COVERED", stateHandler.getBBThreeCovered());
    SmartDashboard.putBoolean("BB FOUR COVERED", stateHandler.getBBFourCovered());

    /* RELEVANT INTAKE STATES */
    SmartDashboard.putString("CURRENT INTAKE ROLLER", stateHandler.getCurrentIntakeRollerSpeed().toString());
    SmartDashboard.putString("DESIRED INTAKE POS", stateHandler.getDesiredIntakeState().toString());
    SmartDashboard.putString("CURRENT INTAKE POS", stateHandler.getCurrentIntakeState().toString());

    /* RELEVANT ARM STATES */
    SmartDashboard.putString("DESIRED ARM STATE", stateHandler.getDesiredArmState().toString());
    SmartDashboard.putString("CURRENT ARM STATE", stateHandler.getCurrentArmState().toString());

    // /* RELEVANT SHOOTER STATES */
    SmartDashboard.putString("DESIRED SHOOTER STATE", stateHandler.getDesiredShootingSpeed().toString());
    SmartDashboard.putString("CURRENT SHOOTER STATE", stateHandler.getCurrentShootingSpeed().toString());

    /* RELEVANT FEEDER STATES */
    SmartDashboard.putString("CURRENT FEEDER DIRECTION", stateHandler.getCurrentFeederSpeed().toString());

    /*POSRPM OFFSET */
    if (stateHandler.isPosRPMTuning()){
        SmartDashboard.putNumber("RPM OFFSET", stateHandler.getRPMOffset());
        SmartDashboard.putNumber("POSITION OFFSET", stateHandler.getPositionOffset());
    }
    
// 

  }
}
