// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateVariables.ArmPosition;
import frc.lib.RobotStateUtils.StateVariables.ArmStates;
import frc.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.lib.LimelightUtil.LimelightInterface;
import frc.lib.RobotStateUtils.StateHandler;

public class ArmSubsystem extends SubsystemBase {

  StateHandler stateHandler = StateHandler.getInstance();
  LimelightInterface limelightInterface = LimelightInterface.getInstance();
  PositionRPMData positionData = PositionRPMData.getInstance();

  private TalonFX armPrimary = new TalonFX(ArmConstants.armMotorPrimaryID);
  private TalonFX armFollower = new TalonFX(ArmConstants.armMotorFollowerID);

  private MotionMagicVoltage motionMagicVoltage;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armPrimary.getConfigurator().apply(new TalonFXConfiguration());
    armFollower.getConfigurator().apply(new TalonFXConfiguration());

    var armConfigs = new TalonFXConfiguration();

    var armSlot0Configs = armConfigs.Slot0;

    armSlot0Configs.kS = ArmConstants.armKS;
    armSlot0Configs.kV = ArmConstants.armKV;
    armSlot0Configs.kP = ArmConstants.armkP;
    armSlot0Configs.kI = ArmConstants.armkI;
    armSlot0Configs.kD = ArmConstants.armkD;

    motionMagicVoltage = new MotionMagicVoltage(0);
    motionMagicVoltage.Slot = 0;

    var motionMagicConfigs = armConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.maxArmVel;
    motionMagicConfigs.MotionMagicAcceleration = ArmConstants.maxArmAccel;
    motionMagicConfigs.MotionMagicJerk = ArmConstants.maxArmJerk;

    armPrimary.getConfigurator().apply(armConfigs, 0.05);
    armFollower.getConfigurator().apply(armConfigs, 0.05);

    // Need to change/test in lab
    armFollower.setControl(new Follower(ArmConstants.armMotorPrimaryID, true));

    zeroArm();
  }

  /**
   * Method to zero the intake arm.
   */
  public void zeroArm() {
    armPrimary.setPosition(0);
  }

  /**
   * Setting the intake to a position/
   * 
   * @param position The position that we are setting the intake is.
   */
  public void setArmPosition(double position) {
    armPrimary.setControl(motionMagicVoltage.withPosition(position * ArmConstants.armRadsToRots)
        .withFeedForward(calculateArmFeedForward()));
  }

  /**
   * Gets the position of the intake arm.
   * 
   * @return The intake position in radians.
   */
  public double getArmPositionRads() {
    return armPrimary.getPosition().getValueAsDouble() * ArmConstants.armRotsToRads;
  }

  /**
   * Gets the position of the intake arm.
   * 
   * @return The intake position in rotations.
   */
  public double getArmPositionRots() {
    return armPrimary.getPosition().getValueAsDouble() * ArmConstants.armRotsToRads;
  }

  /**
   * Calculates the required Feedforward needed for the intake arm.
   * 
   * @return The feedforward value needed by the intake.
   */
  public double calculateArmFeedForward() {
    return IntakeConstants.intakeMaxGravityConstant * Math.cos(getArmPositionRads());
  }

  /**
   * Stops the intake arm motors.
   */
  public void stopIntakeArmMotors() {
    armPrimary.stopMotor();
    armFollower.stopMotor();
  }

  /**
   * Disabling the MotionMagic Control
   */
  public void disableMotionMagic() {
    armPrimary.disable();
    armFollower.disable();
  }

  public boolean isAtArmState(ArmStates armState) {
    return Math
        .abs(getArmPositionRads() - armState.getArmPosition().getAngularSetpoint()) < ArmConstants.armPositionAllowableOffset;
  }

  @Override
  public void periodic() {
    /*
     * By default, whatever the desired position is, we will go to the desired
     * position as commanded.
     * EXCEPTION: shooting. If this is the case, then we will need to modify
     * out arm position to be variable.
     */

    ArmStates desiredArmState = stateHandler.getDesiredArmState();
    double armSetpoint = desiredArmState.getArmPosition().getAngularSetpoint();

    if (desiredArmState == ArmStates.SPEAKER && limelightInterface.hasSpeakerTag() && stateHandler.isHasGamePiece()) {
      armSetpoint = positionData.getDesiredArmPosition(stateHandler.getDistanceToTag());
    }
    else if(desiredArmState == ArmStates.AMP && limelightInterface.hasAmpTag() && stateHandler.isHasGamePiece()){
      armSetpoint = ArmStates.AMP.getArmPosition().getAngularSetpoint();
    }
    else if(desiredArmState == ArmStates.CLIMB){
      armSetpoint = ArmStates.CLIMB.getArmPosition().getAngularSetpoint();
    }

    if(!stateHandler.getBBThreeCovered() && !stateHandler.getBBFourCovered() && stateHandler.getCurrentArmState() == ArmStates.SPEAKER){
      stateHandler.setDesiredArmState(ArmStates.STOWED);
    }
    else if(!stateHandler.isHasGamePiece() && stateHandler.getCurrentArmState() == ArmStates.AMP){
      stateHandler.setDesiredArmState(ArmStates.STOWED);
    }

    /**
     * Set the arm position to whatever is the desired arm position.
     */

    setArmPosition(armSetpoint);

    /**
     * Below lines/logic is used to update the arm's position.
     */
    if (isAtArmState(desiredArmState)) {
      stateHandler.setCurrentArmState(desiredArmState);
    }


  }
}