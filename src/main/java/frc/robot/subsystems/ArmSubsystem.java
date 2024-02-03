// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateVariables.ArmStates;
import frc.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.Constants.ArmConstants;
import frc.lib.LimelightUtil.LimelightInterface;
import frc.lib.RobotStateUtils.StateHandler;

public class ArmSubsystem extends SubsystemBase {

  StateHandler stateHandler = StateHandler.getInstance();
  LimelightInterface limelightInterface = LimelightInterface.getInstance();
  PositionRPMData positionData = PositionRPMData.getInstance();

  private TalonFX armPrimary = new TalonFX(ArmConstants.armMotorPrimaryID, "rio");
  private TalonFX armFollower = new TalonFX(ArmConstants.armMotorFollowerID, "rio");

  private MotionMagicVoltage motionMagicVoltage;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armPrimary.getConfigurator().apply(new TalonFXConfiguration());
    armFollower.getConfigurator().apply(new TalonFXConfiguration());

    var armConfigs = new TalonFXConfiguration();

    armConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var armSlot0Configs = armConfigs.Slot0;

    armSlot0Configs.kS = ArmConstants.armKS;
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
   * Method to zero the arm.
   */
  public void zeroArm() {
    armPrimary.setPosition(0);
  }

  /**
   * Setting the arm to a position in radians.
   * 
   * @param position The radian position to command the arm to.
   */
  public void setArmPosition(double position) {
    armPrimary.setControl(motionMagicVoltage.withPosition(position * ArmConstants.armRadsToRots)
        .withFeedForward(calculateArmFeedForward()));
  }
  
  /**
   * Moving the arm using percent out
   * 
   * @param out percent out speed to run the arm at
   */
  public void setPercentOut(double out){
    armPrimary.set(out);
  }

  /**
   * Gets the position of the arm.
   * 
   * @return The arm position in radians.
   */
  public double getArmPositionRads() {
    return armPrimary.getPosition().getValueAsDouble() * ArmConstants.armRotsToRads;
  }

  /**
   * Gets the position of the arm.
   * 
   * @return The arm position in rotations.
   */
  public double getArmPositionRots() {
    return armPrimary.getPosition().getValueAsDouble();
  }

  /**
   * Calculates the required Feedforward needed for the arm.
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

  public boolean isAtArmState(double desiredSetpoint) {
    return Math.abs(getArmPositionRads() - desiredSetpoint) < ArmConstants.armPositionAllowableOffset;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raw Postion ARM ", armPrimary.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Intake Position Radians", getArmPositionRads());
    /*
     * By default, whatever the desired position is, we will go to the desired
     * position as commanded.
     * EXCEPTION: shooting. If this is the case, then we will need to modify
     * out arm position to be variable.
     */




    //TODO: STATE MACHINE PUT BACK OR SAD
    // ArmStates desiredArmState = stateHandler.getDesiredArmState();
    // double armSetpoint = desiredArmState.getArmPosition().getAngularSetpoint();

    // if (desiredArmState == ArmStates.SPEAKER) {
    //   // subwoofer condition
    //   if ((stateHandler.getHasValidSpeakerTag()
    //       && stateHandler.getDistanceToSpeakerTag() < ArmConstants.SUBWOOFER_THRESHHOLD)
    //       || (!limelightInterface.hasSpeakerTag())) {
    //     armSetpoint = ArmStates.SPEAKER.getArmPosition().getAngularSetpoint();
    //   }
    //   // distance to speaker condition
    //   else if (stateHandler.getHasValidSpeakerTag()) {
    //     armSetpoint = positionData.getDesiredArmPosition(stateHandler.getDistanceToSpeakerTag());
    //   }
    // }

    // /*
    //  * Set the arm position to whatever is the desired arm position.
    //  */
    // setArmPosition(armSetpoint);

    // /*
    //  * Update the arm's position based on the desired setpoint.
    //  */
    // if (isAtArmState(armSetpoint)) {
    //   stateHandler.setCurrentArmState(desiredArmState);
    // }

    if (DriverStation.isDisabled()) {
      disableMotionMagic();
    }

  }
}