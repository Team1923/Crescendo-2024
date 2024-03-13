package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Queue;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;
import frc.robot.lib.StateMachine.StateVariables.FeederSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeRollerSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeStates;
import frc.robot.lib.StateMachine.StateVariables.ShooterSpeeds;

public class FeederSubsystem extends SubsystemBase {
  /* Beam Break initializations. These are DigitalInput objects that return true/false. */
  private DigitalInput beamBreakTwo = new DigitalInput(FeederConstants.beamBreakTwoID);
  private DigitalInput beamBreakThree = new DigitalInput(FeederConstants.beamBreakThreeID);

  // boolean armP =false;
  // boolean llP = false;
  // boolean shooterP = false;

  // ArrayList<String> events = new ArrayList<>();

  /* Instantiate the StateHandler to get useful data on the robot's current state. */
  private StateHandler stateHandler = StateHandler.getInstance();

  /* Initialize the Feeder Motor. */
  private TalonFX feederMotor = new TalonFX(FeederConstants.feederMotorID, "rio");

  /* Construct the feeder subsystem. This will be used to apply any configs to the feeder motor. */
  public FeederSubsystem() {
    /* Wipe the data on the feeder motor. */
    feederMotor.getConfigurator().apply(new TalonFXConfiguration());

    var currentConfigurator = new CurrentLimitsConfigs();
    feederMotor.getConfigurator().refresh(currentConfigurator);
    currentConfigurator.StatorCurrentLimit = CurrentConstants.kStatorCurrentLimit;
    currentConfigurator.StatorCurrentLimitEnable = CurrentConstants.kStatorCurrentLimitEnable;
    feederMotor.getConfigurator().apply(currentConfigurator);
    
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

  /**
   * Setting the stator current limit for the feeder motors.
   */
  public void checkCurrentLimits(){
    if (Math.abs(feederMotor.getStatorCurrent().getValueAsDouble())>(10+CurrentConstants.kStatorCurrentLimit)){
      SmartDashboard.putNumber("Over Stator on feeder", feederMotor.getStatorCurrent().getValueAsDouble());
    }
  }

  @Override
  public void periodic() {
    stateHandler.setBBTwoCovered(getBeamBreakTwo());
    stateHandler.setBBThreeCovered(getBeamBreakThree());

    checkCurrentLimits();

    /**
     * Checks the conditions that 
     */
    // if (stateHandler.getCurrentArmState() == ArmStates.SPEAKER && !armP){
    //   events.add("ARM READY");
    //   armP = true;
    // }
    // if (stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT && !shooterP){
    //   events.add("SHOOTER READY");
    //   shooterP = true;
    // }
    // if (stateHandler.getIsCenteredToTag() && (stateHandler.getDistanceToSpeakerTag() <= LimeLightConstants.speakerLerpUpperBound && stateHandler.getDistanceToSpeakerTag() >= LimeLightConstants.speakerLerpLowerBound) && !llP){
    //   events.add("LL CENTERED and WITHIN DISTANCE");
    //   llP = true;
    // }


    FeederSpeeds desiredFeederSpeed = StateHandler.getInstance().getDesiredFeederSpeed();

    if(stateHandler.getFullEject()){
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }

    else if (stateHandler.getCurrentIntakeState() == IntakeStates.DEPLOYED
        && stateHandler.getDesiredIntakeState() == IntakeStates.DEPLOYED
        && stateHandler.getCurrentIntakeRollerSpeed() == IntakeRollerSpeeds.EJECT) {
      /* HANDLES EJECT CONDITION */
      desiredFeederSpeed = FeederSpeeds.OUTWARD;
    } 
    
    /*
     * Not subwoofer shot
     */
  
    else if (stateHandler.getCurrentArmState() == ArmStates.SPEAKER
        && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT
        && (stateHandler.getIsCenteredToTag()) && 
        (stateHandler.getDistanceToSpeakerTag() <= LimeLightConstants.speakerLerpUpperBound && stateHandler.getDistanceToSpeakerTag() >= LimeLightConstants.speakerLerpLowerBound)) {
      /* CONDITION: ready to sore (center to tag = true on default) */
      desiredFeederSpeed = FeederSpeeds.INWARD;

  
    }

    else if(stateHandler.getCurrentArmState() == ArmStates.PUNT){
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }

    /*
     * Subwoofer shot
     */
    else if(stateHandler.getCurrentArmState() == ArmStates.SPEAKER && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT
    && (stateHandler.getScoreInSubwoofer() || stateHandler.getScoreInReverseSubwoofer())){
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }

    /*
     * edge condition for running intake until note is fully in (already handled eject condition so this shouldn't override?)
     */
    else if (!stateHandler.getBBThreeCovered() && stateHandler.getCurrentIntakeState() == IntakeStates.DEPLOYED && !(stateHandler.getDesiredArmState() == ArmStates.CLIMB) && !stateHandler.getManuallyClimbing()){
        desiredFeederSpeed = FeederSpeeds.INWARD;
    }
    
    // else if (stateHandler.getCurrentArmState() != ArmStates.SPEAKER
    //   && stateHandler.getDesiredShootingSpeed() != ShooterSpeeds.SHOOT
    //   && stateHandler.getDesiredIntakeState() != IntakeStates.DEPLOYED) {
    //     if (stateHandler.getBBFourCovered()) {
    //       desiredFeederSpeed = FeederSpeeds.BACKING;
    //     } else if (!stateHandler.getBBFourCovered() && stateHandler.getCurrentArmState() != ArmStates.AMP) {
    //       desiredFeederSpeed = FeederSpeeds.OFF;
    //     }
    // }

    else if (!stateHandler.getScoreInAmp() &&
      stateHandler.getCurrentArmState() != ArmStates.SPEAKER
      && stateHandler.getDesiredShootingSpeed() != ShooterSpeeds.SHOOT
      && stateHandler.getDesiredIntakeState() != IntakeStates.DEPLOYED) {
        if (stateHandler.getBBFourCovered()) {
          desiredFeederSpeed = FeederSpeeds.BACKING;
        } else if (!stateHandler.getBBFourCovered() && stateHandler.getCurrentArmState() != ArmStates.AMP) {
          desiredFeederSpeed = FeederSpeeds.OFF;
        }
    }

    else if (stateHandler.getScoreInAmp() &&
      stateHandler.getCurrentArmState() != ArmStates.AMP
      && stateHandler.getDesiredShootingSpeed() != ShooterSpeeds.SHOOT
      && stateHandler.getDesiredIntakeState() != IntakeStates.DEPLOYED) {
        if (!stateHandler.getBBFourCovered() && stateHandler.getBBThreeCovered()) {
          desiredFeederSpeed = FeederSpeeds.FORWARD;
        } else if ((stateHandler.getBBFourCovered() || !stateHandler.getBBThreeCovered() && !stateHandler.getBBTwoCovered()) && stateHandler.getCurrentArmState() != ArmStates.AMP) {
          desiredFeederSpeed = FeederSpeeds.OFF;
        }
    }



    
    // /* Set the feeder motor speed to whatever it needs to be. */
    setFeederMotorSpeed(desiredFeederSpeed.getPercentOutputValue().getPercentOut());


    stateHandler.setCurrentFeederSpeed(desiredFeederSpeed);

  }

}