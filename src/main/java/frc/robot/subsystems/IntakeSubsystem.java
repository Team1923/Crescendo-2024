// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateHandler;
import frc.lib.RobotStateUtils.StateVariables.IntakeStates;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private DigitalInput beamBreakOne = new DigitalInput(IntakeConstants.beamBreakOneID);
  private StateHandler stateHandler = StateHandler.getInstance();

  private TalonFX intakeArmPrimary = new TalonFX(Constants.IntakeConstants.intakePrimaryID);
  private TalonFX intakeArmFollower = new TalonFX(Constants.IntakeConstants.intakeFollowerID);

  private TalonFX intakeWheelTop = new TalonFX(Constants.IntakeConstants.intakeWheelTopID);
  private TalonFX intakeWheelBottom = new TalonFX(Constants.IntakeConstants.intakeWheelBottomID);

  private MotionMagicVoltage motionMagicVoltage;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeArmPrimary.getConfigurator().apply(new TalonFXConfiguration());
    intakeArmFollower.getConfigurator().apply(new TalonFXConfiguration());
    intakeWheelTop.getConfigurator().apply(new TalonFXConfiguration());
    intakeWheelBottom.getConfigurator().apply(new TalonFXConfiguration());

    var intakeArmConfigs = new TalonFXConfiguration();

    var intakeArmSlot0Configs = intakeArmConfigs.Slot0;

    motionMagicVoltage = new MotionMagicVoltage(0);
    motionMagicVoltage.Slot = 0;

    // subject all to change
    intakeArmSlot0Configs.kS = Constants.IntakeConstants.intakeKS;
    intakeArmSlot0Configs.kV = Constants.IntakeConstants.intakeKV;
    intakeArmSlot0Configs.kP = Constants.IntakeConstants.intakekP;
    intakeArmSlot0Configs.kI = Constants.IntakeConstants.intakekI;
    intakeArmSlot0Configs.kD = Constants.IntakeConstants.intakekD;

    // motion magic configs
    var motionMagicConfigs = intakeArmConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.IntakeConstants.maxIntakeVel;
    motionMagicConfigs.MotionMagicAcceleration = Constants.IntakeConstants.maxIntakeAccel;
    motionMagicConfigs.MotionMagicJerk = Constants.IntakeConstants.maxIntakeJerk;

    intakeArmPrimary.getConfigurator().apply(intakeArmConfigs, 0.05);
    intakeArmFollower.getConfigurator().apply(intakeArmConfigs, 0.05);

    // Need to change/test in lab
    intakeArmFollower.setControl(new Follower(Constants.IntakeConstants.intakePrimaryID, true));

    zeroIntakeArm();
  }

  /**
   * Method to zero the intake arm.
   */
  public void zeroIntakeArm() {
    intakeArmPrimary.setPosition(0);
  }

  /**
   * Setting the intake to a position/
   * 
   * @param position The position that we are setting the intake is.
   */
  public void setIntakePosition(double position) {
    intakeArmPrimary.setControl(motionMagicVoltage.withPosition(position * Constants.IntakeConstants.intakeRadsToRots)
        .withFeedForward(calculateIntakeFeedForward()));
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
    return IntakeConstants.intakeMaxGravityConstant * Math.cos(getIntakeArmPositionRads());
  }

  /**
   * Sets the speed for the top intake wheel.
   * 
   * @param speed The speed passed in.
   */
  public void setTopWheelSpeed(double speed) {
    intakeWheelTop.set(speed);
  }

  /**
   * Sets the speed for the bottom intake wheel.
   * 
   * @param speed The speed passed in.
   */
  public void setBottomWheelSpeed(double speed) {
    intakeWheelBottom.set(speed);
  }

  /**
   * Stops the intake arm motors.
   */
  public void stopIntakeArmMotors() {
    intakeArmPrimary.stopMotor();
    intakeArmFollower.stopMotor();
  }

  /**
   * Stops the intake wheel motors.
   */
  public void stopIntakeWheels() {
    intakeWheelTop.stopMotor();
    intakeWheelBottom.stopMotor();
  }

  /**
   * Disabling the MotionMagic Control
   */
  public void disableMotionMagic() {
    intakeArmPrimary.disable();
    intakeArmFollower.disable();
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
   * Method to get the digital input reading of BB1.
   * TODO: verify if adding a NOT before the boolean is needed.
   * 
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakOne() {
    return beamBreakOne.get();
  }

  public boolean isAtIntakeState(IntakeStates intakeStates) {
    return Math.abs(getIntakeArmPositionRads() -
        intakeStates.getIntakePosition().getAngularSetpoint()) < IntakeConstants.intakeStateThreshold;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    stateHandler.setBBOneCovered(getBeamBreakOne());

    // check the stator current to know whether or not we are hardstop.

    /*
     * if current position = deployed and BB3 is triggered -> desiredPostion =
     * stowed
     * 
     * if current position != desired position -> move to desired position
     * if current wheel speed != desired wheel speed -> spin at desired wheel speed
     */
  }
}