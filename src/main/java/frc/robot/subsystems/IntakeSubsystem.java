package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CurrentConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.IntakeRollerSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeStates;

public class IntakeSubsystem extends SubsystemBase {
  private DigitalInput beamBreakOne = new DigitalInput(IntakeConstants.beamBreakOneID);
  private StateHandler stateHandler = StateHandler.getInstance();

  private TalonFX intakeArmPrimary = new TalonFX(Constants.IntakeConstants.intakeArmPrimaryID, "rio");
  private TalonFX intakeArmFollower = new TalonFX(Constants.IntakeConstants.intakeArmFollowerID, "rio");

  private TalonFX intakeWheelTop = new TalonFX(Constants.IntakeConstants.intakeWheelTopID, "rio");
  private TalonFX intakeWheelBottom = new TalonFX(Constants.IntakeConstants.intakeWheelBottomID, "rio");

  private MotionMagicVoltage motionMagicVoltage;

  private boolean bb1Crossed = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeWheelTop.getConfigurator().apply(new TalonFXConfiguration());
    intakeWheelBottom.getConfigurator().apply(new TalonFXConfiguration());

    var intakeArmConfigs = new TalonFXConfiguration();

    var intakeArmSlot0Configs = intakeArmConfigs.Slot0;

    intakeArmConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // subject all to change
    intakeArmSlot0Configs.kS = Constants.IntakeConstants.intakeKS;
    intakeArmSlot0Configs.kV = Constants.IntakeConstants.intakekV;
    intakeArmSlot0Configs.kP = Constants.IntakeConstants.intakekP;
    intakeArmSlot0Configs.kI = Constants.IntakeConstants.intakekI;
    intakeArmSlot0Configs.kD = Constants.IntakeConstants.intakekD;

    motionMagicVoltage = new MotionMagicVoltage(0);
    motionMagicVoltage.Slot = 0;

    // motion magic configs
    var motionMagicConfigs = intakeArmConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.IntakeConstants.maxIntakeVel;
    motionMagicConfigs.MotionMagicAcceleration = Constants.IntakeConstants.maxIntakeAccel;
    motionMagicConfigs.MotionMagicJerk = Constants.IntakeConstants.maxIntakeJerk;

    intakeArmPrimary.getConfigurator().apply(intakeArmConfigs);
    intakeArmFollower.getConfigurator().apply(intakeArmConfigs);

    intakeArmPrimary.setInverted(true);
    intakeArmFollower.setInverted(true);

    // Need to change/test in lab
    intakeArmFollower.setControl(new Follower(IntakeConstants.intakeArmPrimaryID, false));

    configCurrentLimit();
    zeroIntakeArm();
  }
/**
 * Sets the current limits for the intake arm motors and intake roller motors.
 */
  public void configCurrentLimit(){
    var customCurrentConfig = new CurrentLimitsConfigs();

  
    intakeWheelTop.getConfigurator().refresh(customCurrentConfig);
    intakeWheelBottom.getConfigurator().refresh(customCurrentConfig);

    intakeArmPrimary.getConfigurator().refresh(customCurrentConfig);
    intakeArmFollower.getConfigurator().refresh(customCurrentConfig);

    customCurrentConfig.StatorCurrentLimit = CurrentConstants.kStatorCurrentLimit;
    customCurrentConfig.StatorCurrentLimitEnable = CurrentConstants.kStatorCurrentLimitEnable;

    intakeWheelTop.getConfigurator().apply(customCurrentConfig);
    intakeWheelBottom.getConfigurator().apply(customCurrentConfig);

    intakeArmFollower.getConfigurator().apply(customCurrentConfig);
    intakeArmPrimary.getConfigurator().apply(customCurrentConfig);
  }

  /**
   * Method to zero the intake arm.
   */
  public void zeroIntakeArm() {
    intakeArmPrimary.setPosition(0);
    intakeArmFollower.setPosition(0);
  }

  /**
   * Setting the intake to a position in radians.
   * 
   * @param position The radian value the intake is commanded to.
   */
  public void setIntakePosition(double position) {
    intakeArmPrimary.setControl(motionMagicVoltage.withPosition(position * Constants.IntakeConstants.intakeRadsToRots)
        .withFeedForward(calculateIntakeFeedForward()));
  }

  /**
   * Moving the intake arm using percent out
   * 
   * @param out percent out speed to run the intake arm at
   */
  public void setIntakeArmPercentOut(double out) {
    intakeArmPrimary.set(out);
  }

  /**
   * Gets the position of the intake arm.
   * 
   * @return The intake position in radians.
   */
  public double getIntakeArmPositionRads() {
    return intakeArmPrimary.getPosition().getValueAsDouble() * IntakeConstants.intakeRotsToRads;
  }

  /**
   * Gets the position of the intake arm.
   * 
   * @return The intake position in rotations.
   */
  public double getIntakeArmPositionRots() {
    return intakeArmPrimary.getPosition().getValueAsDouble();
  }

  /**
   * Calculates the required Feedforward needed for the intake arm.
   * 
   * @return The feedforward value needed by the intake.
   */
  public double calculateIntakeFeedForward() {
    return IntakeConstants.intakeMaxGravityConstant * Math.sin(getIntakeArmPositionRads());
  }

  /**
   * Sets the speed for the top intake wheel.(Make these negative)
   * 
   * @param speed The speed passed in.
   *              //
   */
  public void setRollerWheelSpeed(double topSpeed, double BottomSpeed) {
    intakeWheelTop.set(topSpeed);
    intakeWheelBottom.set(BottomSpeed);
  }

  /**
   * Stops the intake arm motors.
   */
  public void stopIntakeArmMotors() {
    intakeArmPrimary.set(0);
    intakeArmFollower.set(0);
  }

  /**
   * Stops the intake wheel motors.
   * //
   */
  public void stopIntakeWheels() {
    intakeWheelTop.set(0);
    intakeWheelBottom.set(0);
  }

  /**
   * Gets the current being drawn by the Intake Arm.
   * 
   * @return The raw stator current being drawn by the intake arm.
   */
  public double getRawIntakeArmCurrent() {
    return intakeArmPrimary.getTorqueCurrent().getValueAsDouble();
  }

  /**
   * Method to get the digital input reading of BB1. *
   * 
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakOne() {
    return !beamBreakOne.get();
  }

  /**
   * Determines wheter or not we are at intake position.
   * @param intakeStates The intake state we pass in. 
   * @return A boolean, true or false, if we are or are not at the intake state.
   */
  public boolean isAtIntakeState(IntakeStates intakeStates) {
    return Math.abs(getIntakeArmPositionRads() -
        intakeStates.getIntakePosition().getAngularSetpoint()) < IntakeConstants.intakePositionAllowableOffset;
  }

  public void setIntakeArmCoast() {
    intakeArmPrimary.setNeutralMode(NeutralModeValue.Coast);
    intakeArmFollower.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setIntakeArmBrake() {
    intakeArmPrimary.setNeutralMode(NeutralModeValue.Brake);
    intakeArmFollower.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Checks the Stator Current limit for the intake motors.
   */
  public void checkCurrentLimits(){
    if (Math.abs(intakeArmPrimary.getStatorCurrent().getValueAsDouble())>(10+CurrentConstants.kStatorCurrentLimit)){
      SmartDashboard.putNumber("Over Stator on intakeArm", intakeArmPrimary.getStatorCurrent().getValueAsDouble());
    }
    if (Math.abs(intakeWheelTop.getStatorCurrent().getValueAsDouble())>(10+CurrentConstants.kStatorCurrentLimit)){
      SmartDashboard.putNumber("Over Stator on intakeWheel", intakeWheelTop.getStatorCurrent().getValueAsDouble());
    }
  }


  @Override
  public void periodic() {
    if (!Utils.isSimulation()){
      stateHandler.setBBOneCovered(getBeamBreakOne());
    }

    /* Pull the desired intake state from StateHandler */
    IntakeStates desiredIntakeState = stateHandler.getDesiredIntakeState();
    IntakeRollerSpeeds desiredRollerSpeedState = stateHandler.getDesiredIntakeRollerSpeed();

    /* Get the corresponding numerical values */
    double intakeSetpoint = desiredIntakeState.getIntakePosition().getAngularSetpoint();
    double rollerSpeed = desiredRollerSpeedState.getPercentOutputValue().getPercentOut();

    /* CASE #1: running full emergency eject */
    if(stateHandler.getFullEject()){
      rollerSpeed = IntakeRollerSpeeds.INTAKE.getPercentOutputValue().getPercentOut();
    }
    
    /* CASE #2: can only run intake rollers when eject conditions are satisfied */
    if (stateHandler.getCurrentIntakeState() != IntakeStates.DEPLOYED
        && desiredIntakeState == IntakeStates.DEPLOYED
        && desiredRollerSpeedState == IntakeRollerSpeeds.EJECT) {
      rollerSpeed = IntakeRollerSpeeds.OFF.getPercentOutputValue().getPercentOut();
    }

    /* CASE #3: prevent operator from bringing up intake until completition of intake command */
    if (stateHandler.getBBOneCovered() 
      && stateHandler.getDesiredIntakeRollerSpeed() != IntakeRollerSpeeds.EJECT) {
      bb1Crossed = true;
    }

    /* Boolean value changes based on BB3 and !BB1 */
    if (stateHandler.getBBThreeCovered() && !stateHandler.getBBOneCovered()) {
      bb1Crossed = false;
    }

    /* CASE #4: Leave the intake down if a game piece is within the robot */
    if (stateHandler.getDesiredIntakeState() == IntakeStates.STOWED
        && !stateHandler.getBBThreeCovered() && bb1Crossed) {
      rollerSpeed = IntakeRollerSpeeds.INTAKE.getPercentOutputValue().getPercentOut();
      intakeSetpoint = IntakeStates.DEPLOYED.getIntakePosition().getAngularSetpoint();
    } 

    /* Set the desired intake position + setpoints accordingly. */
    setIntakePosition(intakeSetpoint);
    setRollerWheelSpeed(rollerSpeed, rollerSpeed);

    /* Update the current state once the desired state has been reached. */
    if (isAtIntakeState(desiredIntakeState)) {
      stateHandler.setCurrentIntakeState(desiredIntakeState);
    }

    stateHandler.setCurrentIntakeRollerSpeed(desiredRollerSpeedState);

  }
}
