// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateHandler;
import frc.lib.RobotStateUtils.StateVariables;
import frc.lib.RobotStateUtils.StateVariables.ShooterSpeeds;
import frc.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX shooterTop = new TalonFX(ShooterConstants.shooterMotorPrimaryID, "rio");
  private TalonFX shooterBottom = new TalonFX(ShooterConstants.shooterMotorFollowerID, "rio");
  private DigitalInput beamBreakFour = new DigitalInput(ShooterConstants.beamBreakFourID);
  private StateHandler stateHandler = StateHandler.getInstance();

  PositionRPMData rpmData = PositionRPMData.getInstance();

  private final MotionMagicVelocityVoltage motionMagicVelVoltage;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    shooterTop.getConfigurator().apply(new TalonFXConfiguration());
    shooterBottom.getConfigurator().apply(new TalonFXConfiguration());


    var shooterFXConfig = new TalonFXConfiguration();

    /**
     * Rough values, need to tune them to final robot.
     */
    var shooterMotorConfig = shooterFXConfig.Slot0;
    shooterMotorConfig.kS = ShooterConstants.shooterKS;
    shooterMotorConfig.kV = ShooterConstants.shooterKV;
    shooterMotorConfig.kA = ShooterConstants.shooterKA;
    shooterMotorConfig.kP = ShooterConstants.shooterKP;
    shooterMotorConfig.kI = ShooterConstants.shooterKI;
    shooterMotorConfig.kD = ShooterConstants.shooterKD;

    motionMagicVelVoltage = new MotionMagicVelocityVoltage(0);
    motionMagicVelVoltage.Slot = 0;

    var shooterMotionMagicConfig = shooterFXConfig.MotionMagic;
    shooterMotionMagicConfig.MotionMagicAcceleration = 100; //TODO: Tune
    shooterMotionMagicConfig.MotionMagicJerk = 1000; //TODO: Tune

    shooterTop.getConfigurator().apply(shooterFXConfig, 0.05);
    shooterBottom.getConfigurator().apply(shooterFXConfig, 0.05);

    shooterBottom.setControl(new Follower(ShooterConstants.shooterMotorPrimaryID, false));

    shooterTop.setNeutralMode(NeutralModeValue.Brake);
    shooterBottom.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Method to set speed of shooter motors, using velocity closed loop control.
   * 
   * @param velocityP The speed, in RPM, passed into the top motor.
   * @param velocityF The speed, in RPM, passed into the bottom motor.
   */
  public void setVelocities(double velocityP, double velocityF) {
    shooterTop.setControl(motionMagicVelVoltage.withVelocity(velocityP * ShooterConstants.shooterRPMToRPS));
    shooterBottom.setControl(motionMagicVelVoltage.withVelocity(velocityF * ShooterConstants.shooterRPMToRPS));
  }

  /**
   * Method to get the digital input reading of BB4.
   * TODO: verify if adding a NOT before the boolean is needed.
   * 
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakFour() {
    return !beamBreakFour.get();
  }

  /**
   * Returns the velocity in RPM of the top shooter motor.
   * 
   * @return Velocity in RPM.
   */
  public double getTopRPM() {
    return shooterTop.getVelocity().getValueAsDouble() * ShooterConstants.shooterRPSToRPM;
  }

  /**
   * Returns the velocity in RPM of the bottom shooter motor.
   * 
   * @return Velocity in RPM.
   */
  public double getBottomRPM() {
    return shooterBottom.getVelocity().getValueAsDouble() * 60;
  }

  /**
   * Stops the motors.
   */
  public void stopMotors() {
    shooterTop.stopMotor();
    shooterBottom.stopMotor();
  }

  public boolean isAtShooterSpeed(double desiredSetpoint) {
    return Math.abs(getTopRPM() - desiredSetpoint) < ShooterConstants.shooterSpeedThreshold
        && Math.abs(getBottomRPM() - desiredSetpoint) < ShooterConstants.shooterSpeedThreshold;
  }

  @Override
  public void periodic() {
    stateHandler.setBBFourCovered(getBeamBreakFour());

    SmartDashboard.putNumber("Raw RPS TOP SHOOTER", shooterTop.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Raw RPS BOTTOM SHOOTER", shooterBottom.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("RPM TOP SHOOTER", getTopRPM());
    SmartDashboard.putNumber("RPM BOTTOM SHOOTER", getBottomRPM());

    //TODO: STATE MACHINE PUT BACK OR SAD
    // ShooterSpeeds desiredShooterSpeedState = stateHandler.getDesiredShootingSpeed();
    // double desiredShooterSpeed = desiredShooterSpeedState.getRPMValue().getRPM();

    // if (desiredShooterSpeedState == ShooterSpeeds.SHOOT) {
    //   /* If at subwoofer, then the desired shot speed is the preset for the subwoofer shot. */
    //   if ((stateHandler.getHasValidSpeakerTag()
    //       && stateHandler.getDistanceToSpeakerTag() < ArmConstants.SUBWOOFER_THRESHHOLD)
    //       || (!stateHandler.getHasValidSpeakerTag())) {
    //     desiredShooterSpeed = ShooterSpeeds.SHOOT.getRPMValue().getRPM();
    //   }
    //   /* If we have a valid tag, then get positional data. */
    //   else if (stateHandler.getHasValidSpeakerTag()
    //       && stateHandler.getDistanceToSpeakerTag() > ArmConstants.SUBWOOFER_THRESHHOLD) {
    //     desiredShooterSpeed = rpmData.getDesiredShooterRPM(stateHandler.getDistanceToSpeakerTag());
    //   }
    // } else if (stateHandler.getBBThreeCovered() && stateHandler.getBBTwoCovered()) {
    //   /* If you have a game piece, start ramping up the shooter speed. */
    //   desiredShooterSpeed = StateVariables.ShooterSpeeds.RAMP.getRPMValue().getRPM();
    // } 

    // /* Set the desired velocity of the shooter wheels. */
    // setVelocities(desiredShooterSpeed, desiredShooterSpeed);


    // if (isAtShooterSpeed(desiredShooterSpeed)) {
    //   stateHandler.setCurrentShootingSpeed(desiredShooterSpeedState);
    // }
  }
}