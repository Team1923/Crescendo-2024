package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CurrentConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.lib.Limelight.LimelightInterface;
import frc.robot.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;

public class ArmSubsystem extends SubsystemBase {
  /* Helper class instantiations to get useful data from the robot. */
  StateHandler stateHandler = StateHandler.getInstance();
  LimelightInterface limelightInterface = LimelightInterface.getInstance();
  PositionRPMData positionData = PositionRPMData.getInstance();

  /* Motor Instantiations */
  private TalonFX armPrimary = new TalonFX(ArmConstants.armMotorPrimaryID, "rio");
  private TalonFX armFollower = new TalonFX(ArmConstants.armMotorFollowerID, "rio");
  private double maxArmAngle = -1000;
  private double minArmAngle = 1000;

  /* Motion Magic Voltage Object - Used to command the arm's position. */
  private MotionMagicVoltage motionMagicVoltage;

  /* Constructor of ArmSubsystem. Used for setting up motors & configurations. */
  public ArmSubsystem() {
    /*
     * Create a new configuration for the arm. This is the object
     * that will be used in order to set up the relevant
     * motion constants for the arm (PID, etc.)
     */
    var armConfigs = new TalonFXConfiguration();

    /* Set up the motor to initially be in Brake mode. */
    armConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /* Out of the 3 possible slots for PID profiles, we will use the default (0). */
    var armSlot0Configs = armConfigs.Slot0;

    /* Set up the configuration's relevant constants. */
    armSlot0Configs.kS = ArmConstants.armKS;
    armSlot0Configs.kP = ArmConstants.armkP;
    armSlot0Configs.kI = ArmConstants.armkI;
    armSlot0Configs.kD = ArmConstants.armkD;

    /* Instantiate the MotionMagicVoltage object. */
    motionMagicVoltage = new MotionMagicVoltage(0);
    motionMagicVoltage.Slot = 0;

    /* Configure the MotionMagic configuration (vel, accel, and jerk). */
    var motionMagicConfigs = armConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.maxArmVel;
    motionMagicConfigs.MotionMagicAcceleration = ArmConstants.maxArmAccel;
    motionMagicConfigs.MotionMagicJerk = ArmConstants.maxArmJerk;

    /* Apply the generated configuration to the motors. */
    armPrimary.getConfigurator().apply(armConfigs);
    armFollower.getConfigurator().apply(armConfigs);

    /* Set the motor to follow the primary motor & oppose its direction. */
    armFollower.setControl(new Follower(ArmConstants.armMotorPrimaryID, true));

    /* Finally, zero the arm so that its STOW position = 0 rads. */
    zeroArm();
  }
  
  /**
   * Limits the stator current for the intake motors as necessary. 
   */
   public void configCurrentLimit(){
    var customCurrentConfig = new CurrentLimitsConfigs();

  
    armPrimary.getConfigurator().refresh(customCurrentConfig);
    armFollower.getConfigurator().refresh(customCurrentConfig);

    customCurrentConfig.StatorCurrentLimit = CurrentConstants.kStatorCurrentLimit;
    customCurrentConfig.StatorCurrentLimitEnable = CurrentConstants.kStatorCurrentLimitEnable;


    armFollower.getConfigurator().apply(customCurrentConfig);
    armPrimary.getConfigurator().apply(customCurrentConfig);
  }

  /**
   * Method to zero the arm. All that we do is override the
   * motor's position to 0 rots/rads.
   */
  public void zeroArm() {
    armPrimary.setPosition(0);
  }

  /**
   * Set the arm to a position in radians.
   * 
   * @param position The radian position to command the arm to.
   */
  public void setArmPosition(double position) {
    armPrimary.setControl(motionMagicVoltage.withPosition((position) * ArmConstants.armRadsToRots )
        .withFeedForward(calculateArmFeedForward()));
  }

  /**
   * Move the arm using percent output. Primarily used for testing purposes.
   * 
   * @param out percent out speed to run the arm at
   */
  public void setPercentOut(double out) {
    armPrimary.set(out);
  }

  /**
   * Get the position of the arm from the encoder reading.
   * 
   * @return The arm position in radians.
   */
  public double getArmPositionRads() {
    return armPrimary.getPosition().getValueAsDouble() * ArmConstants.armRotsToRads;
  }

  /**
   * Gets the position of the arm, converted from the encoder reading.
   * 
   * @return The arm position in rotations.
   */
  public double getArmPositionRots() {
    return armPrimary.getPosition().getValueAsDouble();
  }

  /**
   * Calculates the required Feedforward needed for the arm. This model
   * largely follows the Arm Feedforward model suggested by WPILib.
   * 
   * @return The feedforward value needed by the arm.
   */
  public double calculateArmFeedForward() {
    return ArmConstants.armMaxGravityConstant * Math.cos(getArmPositionRads());
  }

  /**
   * Stops the arm motors.
   */
  public void stopArmMotors() {
    armPrimary.set(0);
    armFollower.set(0);
  }

  /**
   * Method to determine if the motor has arrived to the commanded state.
   * 
   * @param desiredSetpoint the angular setpoint of the commanded state.
   * @return a boolean to determine if the motor's current position is equal to
   *         the position
   *         that is specified by the desired state.
   */
  public boolean isAtArmState(double desiredSetpoint) {
    return Math.abs(getArmPositionRads() - desiredSetpoint) < ArmConstants.armPositionAllowableOffset; 
  }

  /**
   * Method to set the neutral mode of the motors to COAST.
   */
  public void setArmCoast() {
    armPrimary.setNeutralMode(NeutralModeValue.Coast);
    armPrimary.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * Method to set the neutral mode of the motors to BRAKE.
   */
  public void setArmBrake() {
    armPrimary.setNeutralMode(NeutralModeValue.Brake);
    armPrimary.setNeutralMode(NeutralModeValue.Brake);
  }

  public void checkCurrentLimits(){
    if (Math.abs(armPrimary.getStatorCurrent().getValueAsDouble())>(10+CurrentConstants.kStatorCurrentLimit)){
      SmartDashboard.putNumber("Over Stator on arm", armPrimary.getStatorCurrent().getValueAsDouble());
    }
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Raw Postion ARM Primary ", armPrimary.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Raw Postion ARM Follower ", armFollower.getPosition().getValueAsDouble());

    /* Begin by pulling the desires arm state from State Handler */
    ArmStates desiredArmState = stateHandler.getDesiredArmState();

    /* Get the corresponding angular setpoint (the actual numerical value) */
    double armSetpoint = desiredArmState.getArmPosition().getAngularSetpoint();

    /* CASE #1: Scoring into the SPEAKER */
    if (desiredArmState == ArmStates.SPEAKER) {
      /* Scoring into the subwoofer - use the default preset value */
      if (stateHandler.getScoreInSubwoofer()) {
        armSetpoint = ArmStates.SPEAKER.getArmPosition().getAngularSetpoint() + (stateHandler.isPosRPMTuning() ? stateHandler.getPositionOffset() : 0);
      }
      /* Scoring into the subwoofer (reverse) - use the default preset value */
      else if(stateHandler.getScoreInReverseSubwoofer()) {
        armSetpoint = ArmStates.REVERSE_SUBWOOFER.getArmPosition().getAngularSetpoint();
      }
      /* Scoring from ranged - use data from HashMap to get the arm position. */
      else if (stateHandler.getHasValidSpeakerTag()) {
        armSetpoint = positionData.getSpeakerDesiredArmPosition(stateHandler.getDistanceToSpeakerTag());
      } else if (DriverStation.isAutonomousEnabled() && !stateHandler.getHasValidSpeakerTag()) {
          armSetpoint = positionData.getSpeakerDesiredArmPosition(stateHandler.getCoveredSpeakerTagDistance());
      }
      /* If the tag is lost in view, preserve the arm position. */
      else {
        armSetpoint = getArmPositionRads();
      }
    }

    /* CASE #2: Scoring into the TRAP */
    else if (desiredArmState == ArmStates.TRAP) {
      /* Need to pull the arm position from the HashMap */
      if (stateHandler.getHasValidTrapTag()) {
        armSetpoint = positionData.getTrapDesiredArmPosition(stateHandler.getDistanceToTrapTag());
      }
      /* Preserve the arm position if the trap tag is lost. */
      else {
        armSetpoint = getArmPositionRads();
      }
    }
   
    /* Set the arm condition - NOTE: do not do this when manually climbing! */
    if (!stateHandler.getManuallyClimbing()) {
          setArmPosition(armSetpoint);
    }

    /* Update the arm's position based on the desired setpoint. */
    if (isAtArmState(armSetpoint)) {
      stateHandler.setCurrentArmState(desiredArmState);
    }

    // /* Some interesting logic for detecting if arm ia at position */
    // if(stateHandler.getCurrentArmState() == ArmStates.SPEAKER && stateHandler.getDesiredArmState() == ArmStates.SPEAKER) {
    //   if(Math.abs(getArmPositionRads()) < Math.abs(minArmAngle)) {
    //     minArmAngle = getArmPositionRads();
    //   }
    //   else if(Math.abs(getArmPositionRads()) > Math.abs(maxArmAngle)) {
    //     maxArmAngle = getArmPositionRads();
    //   }
    // }

    // else if( minArmAngle != 1000 && maxArmAngle != -1000) {
    //   System.out.println("Min Arm Angle When At Setpoint" +  minArmAngle);
    //   System.out.println("Max Arm Angle At Setpoint" + maxArmAngle);
    //   minArmAngle = 1000;
    //   maxArmAngle =-1000;
    // }


  }
}
    

