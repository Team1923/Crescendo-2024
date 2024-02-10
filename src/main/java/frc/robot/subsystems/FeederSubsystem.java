package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateVariables.ArmStates;
import frc.lib.RobotStateUtils.StateVariables.FeederSpeeds;
import frc.lib.RobotStateUtils.StateVariables.IntakeRollerSpeeds;
import frc.lib.RobotStateUtils.StateVariables.IntakeStates;
import frc.lib.RobotStateUtils.StateVariables.ShooterSpeeds;
import frc.robot.Constants.FeederConstants;
import frc.lib.RobotStateUtils.StateHandler;

public class FeederSubsystem extends SubsystemBase {
  /* Beam Break initializations. These are DigitalInput objects that return true/false. */
  private DigitalInput beamBreakTwo = new DigitalInput(FeederConstants.beamBreakTwoID);
  private DigitalInput beamBreakThree = new DigitalInput(FeederConstants.beamBreakThreeID);

  /* Instantiate the StateHandler to get useful data on the robot's current state. */
  private StateHandler stateHandler = StateHandler.getInstance();

  /* Initialize the Feeder Motor. */
  private TalonFX feederMotor = new TalonFX(FeederConstants.feederMotorID, "rio");

  /* Construct the feeder subsystem. This will be used to apply any configs to the feeder motor. */
  public FeederSubsystem() {
    /* Wipe the data on the feeder motor. */
    feederMotor.getConfigurator().apply(new TalonFXConfiguration());
    
    /* Set the NeutralMode of the FeederMotor to be BRAKE. */
    feederMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Set the speed of the feeder motor (percent output based control).
   * @param s The desired output speed as a decimal-representation of a percent.
   */
  public void setFeederMotorSpeed(double s) {
    feederMotor.set(s);
  }

  /**
   * This method stops the feeder motor..
   */
  public void stopFeederMotor() {
    feederMotor.stopMotor();
  }

  /**
   * Method to get the digital input reading of BB2.   * 
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakTwo() {
    return !beamBreakTwo.get();
  }

  /**
   * Method to get the digital input reading of BB3.   * 
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakThree() {
    return !beamBreakThree.get();
  }

  @Override
  public void periodic() {
    stateHandler.setBBTwoCovered(getBeamBreakTwo());
    stateHandler.setBBThreeCovered(getBeamBreakThree());

    FeederSpeeds desiredFeederSpeed = StateHandler.getInstance().getDesiredFeederSpeed();

    if (stateHandler.getCurrentIntakeState() == IntakeStates.DEPLOYED
        && stateHandler.getDesiredIntakeState() == IntakeStates.DEPLOYED
        && stateHandler.getCurrentIntakeRollerSpeed() == IntakeRollerSpeeds.EJECT) {
      /* HANDLES EJECT CONDITION */
      desiredFeederSpeed = FeederSpeeds.OUTWARD;
    } 
    
    else if (stateHandler.getCurrentArmState() == ArmStates.SPEAKER
        && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT
        && stateHandler.getIsCenteredToTag()) {
      /* CONDITION: ready to sore (center to tag = true on default) */
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }
    
    else if (stateHandler.getCurrentArmState() != ArmStates.SPEAKER
      && stateHandler.getDesiredShootingSpeed() != ShooterSpeeds.SHOOT
      && stateHandler.getDesiredIntakeState() != IntakeStates.DEPLOYED) {
        if (stateHandler.getBBFourCovered()) {
          desiredFeederSpeed = FeederSpeeds.BACKING;
        } else if (!stateHandler.getBBFourCovered() && stateHandler.getCurrentArmState() != ArmStates.AMP) {
          desiredFeederSpeed = FeederSpeeds.OFF;
        }
    }
    
    // /* Set the feeder motor speed to whatever it needs to be. */
    setFeederMotorSpeed(desiredFeederSpeed.getPercentOutputValue().getPercentOut());


    stateHandler.setCurrentFeederSpeed(desiredFeederSpeed);

  }

}