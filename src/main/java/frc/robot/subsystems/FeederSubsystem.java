package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Queue;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.jsontype.impl.StdTypeResolverBuilder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
   * 
   * @param s The desired output speed as a decimal-representation of a percent.
   */
  public void setFeederMotorSpeed(double s) {
    feederMotor.set(s);
  }

  /**
   * This method stops the feeder motor.
   */
  public void stopFeederMotor() {
    feederMotor.stopMotor();
  }

  /**
   * Method to get the digital input reading of BB2.
   * 
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakTwo() {
    return !beamBreakTwo.get();
  }

  /**
   * Method to get the digital input reading of BB3. *
   * 
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakThree() {
    return !beamBreakThree.get();
  }

  /**
   * Setting the stator current limit for the feeder motors.
   */
  public void checkCurrentLimits() {
    if (Math.abs(feederMotor.getStatorCurrent().getValueAsDouble()) > (10 + CurrentConstants.kStatorCurrentLimit)) {
      SmartDashboard.putNumber("Over Stator on feeder", feederMotor.getStatorCurrent().getValueAsDouble());
    }
  }

  @Override
  public void periodic() {
    /* When the robot is not simulating code, use the value from the beambreaks */
    if (!Utils.isSimulation()){
      stateHandler.setBBTwoCovered(getBeamBreakTwo());
      stateHandler.setBBThreeCovered(getBeamBreakThree());
    }

    /* Begin by pulling the desired feeder wheel speed from the State Handler */
    FeederSpeeds desiredFeederSpeed = StateHandler.getInstance().getDesiredFeederSpeed();

    /* CASE #1: FULL/EMERGENCY EJECT */
    if (stateHandler.getFullEject()) {
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }

    /* CASE #2: Eject via INTAKE */
    else if (stateHandler.getCurrentIntakeState() == IntakeStates.DEPLOYED
        && stateHandler.getDesiredIntakeState() == IntakeStates.DEPLOYED
        && stateHandler.getCurrentIntakeRollerSpeed() == IntakeRollerSpeeds.EJECT) {  
        desiredFeederSpeed = FeederSpeeds.OUTWARD;
    }

    /* CASE #3: RANGED SHOOTING */
    else if (stateHandler.getCurrentArmState() == ArmStates.SPEAKER
        && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT
        && ((stateHandler.getIsCenteredToTag() &&
        (stateHandler.getDistanceToSpeakerTag() <= LimeLightConstants.speakerLerpUpperBound
        && stateHandler.getDistanceToSpeakerTag() >= LimeLightConstants.speakerLerpLowerBound)) || stateHandler.getAutoOverride())
        && stateHandler.getOperatorInputTimingGood()) {
          desiredFeederSpeed = FeederSpeeds.INWARD;
    }

    /* CASE #4: TRAP SHOOTING */
    else if(stateHandler.getCurrentArmState() == ArmStates.TRAP
    && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.TRAP

    && stateHandler.getOperatorInputTimingGood()) {
      if(stateHandler.getHasValidTrapTag()) {
        if((stateHandler.getIsCenteredToTag()) && (stateHandler.getDistanceToTrapTag() <= LimeLightConstants.trapLerpUpperBound 
            && stateHandler.getDistanceToTrapTag() >= LimeLightConstants.trapLerpLowerBound)) {
              desiredFeederSpeed = FeederSpeeds.INWARD;
            }
      }
      else{
        desiredFeederSpeed = FeederSpeeds.INWARD;
      }
    }

    /* CASE #5: Punt Shot */
    else if (stateHandler.getCurrentArmState() == ArmStates.PUNT_HIGH &&
        stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.PUNT_SHOT_HIGH) {
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }

    else if(stateHandler.getCurrentArmState() == ArmStates.PUNT_LOW &&
        stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.PUNT_SHOT_LOW){
          desiredFeederSpeed = FeederSpeeds.INWARD;
        }

    /* Case #6: Ungaurdable Shot */
    else if (stateHandler.getCurrentArmState() == ArmStates.UNGUARDABLE &&
        stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.UNGUARDABLE_SHOT
        && stateHandler.getOperatorInputTimingGood()) {
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }

    /* Case #7: Subwoofer shot. */
    else if (stateHandler.getCurrentArmState() == ArmStates.SPEAKER
        && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT
        && (stateHandler.getScoreInSubwoofer() || stateHandler.getScoreInReverseSubwoofer())) {
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }

    /* Case #8: Front Amp Shot */
    else if(stateHandler.getCurrentArmState() == ArmStates.FRONT_AMP
    && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.FRONT_AMP_SHOT
    && stateHandler.getWantFrontAmp()){
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }

    /* Case #8: continuously run intake when deployed and game piece is within robot */
    else if (!stateHandler.getBBThreeCovered() && stateHandler.getCurrentIntakeState() == IntakeStates.DEPLOYED
        && !(stateHandler.getDesiredArmState() == ArmStates.CLIMB) && !stateHandler.getManuallyClimbing()) {
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }

    /* Case 9 & 10: Shuffle note depending on desired arm position */
    else if (!stateHandler.getScoreInAmp() &&
        stateHandler.getCurrentArmState() != ArmStates.SPEAKER
        && stateHandler.getDesiredShootingSpeed() != ShooterSpeeds.SHOOT
        && stateHandler.getDesiredIntakeState() != IntakeStates.DEPLOYED) {
      if (stateHandler.getBBFourCovered()) {
        desiredFeederSpeed = FeederSpeeds.BACKING;
      } 
      else if(!stateHandler.getBBThreeCovered() && stateHandler.getBBTwoCovered()){
        desiredFeederSpeed = FeederSpeeds.FORWARD;
      }
      else if (!stateHandler.getBBFourCovered() && stateHandler.getCurrentArmState() != ArmStates.AMP) {
        desiredFeederSpeed = FeederSpeeds.OFF;
      }
    }

    else if (stateHandler.getScoreInAmp() &&
        stateHandler.getCurrentArmState() != ArmStates.AMP
        && stateHandler.getDesiredShootingSpeed() != ShooterSpeeds.SHOOT
        && stateHandler.getDesiredIntakeState() != IntakeStates.DEPLOYED) {
      if (!stateHandler.getBBFourCovered() && stateHandler.getBBThreeCovered()) {
        desiredFeederSpeed = FeederSpeeds.FORWARD;
      } else if ((stateHandler.getBBFourCovered()
          || !stateHandler.getBBThreeCovered() && !stateHandler.getBBTwoCovered())
          && stateHandler.getCurrentArmState() != ArmStates.AMP) {
        desiredFeederSpeed = FeederSpeeds.OFF;
      }
    }

    /* Set the feeder motor speed to whatever it needs to be. */
    setFeederMotorSpeed(desiredFeederSpeed.getPercentOutputValue().getPercentOut());

    /* Update the State Handler with the current feeder speed */
    stateHandler.setCurrentFeederSpeed(desiredFeederSpeed);

  }

}