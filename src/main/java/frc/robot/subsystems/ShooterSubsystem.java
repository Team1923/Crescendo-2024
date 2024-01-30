// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateHandler;
import frc.lib.RobotStateUtils.StateVariables.ShooterSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX shooterTop = new TalonFX(ShooterConstants.shooterMotorPrimaryID);
  private TalonFX shooterBottom = new TalonFX(ShooterConstants.shooterMotorFollowerID);
  private DigitalInput beamBreakFour = new DigitalInput(ShooterConstants.beamBreakFourID);
  private StateHandler stateHandler = StateHandler.getInstance();

 private final VelocityVoltage m_velocitytop = new VelocityVoltage(0);
  private final VelocityVoltage m_velocitybottom = new VelocityVoltage(0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    shooterTop.getConfigurator().apply(new TalonFXConfiguration());
    shooterBottom.getConfigurator().apply(new TalonFXConfiguration());

    shooterTop.setInverted(false);

    var slot0Configs = new Slot0Configs();
    var slot1Configs = new Slot1Configs();
    /**
     * Rough values, need to tune them to final robot.
     */
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11; 
    slot0Configs.kI = 0.48; 
    slot0Configs.kD = 0.01; 

     /**
     * We need to change this for TOP Motor
     */
    slot1Configs.kS = 0;
    slot1Configs.kV = 0.11; // try increasing to get closer to goal at 6000
    slot1Configs.kP = 0.11;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0;

    shooterTop.getConfigurator().apply(slot1Configs, 0.05);
    shooterBottom.getConfigurator().apply(slot0Configs, 0.05);
  }

  /**
   * Method to set speed of shooter motors, using velocity closed loop control.
   * @param velocity The speed, in RPM, passed into the motors.
   */
  public void set(double velocityP, double velocityF){

    m_velocitytop.Slot = 1;
    m_velocitybottom.Slot = 0;

    shooterTop.setControl(m_velocitytop.withVelocity(velocityP));
    shooterBottom.setControl(m_velocitybottom.withVelocity(velocityF));
  }

  /**
   * Method to get the digital input reading of BB4.
   * TODO: verify if adding a NOT before the boolean is needed.
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakFour() {
    return beamBreakFour.get();
  }

  /**
   * Returns the velocity in RPM of the top shooter motor.
   * @return Velocity in RPM.
   */
  public double getVelocityTop(){
    return shooterTop.getVelocity().getValueAsDouble() * 60;
  }

  /**
   * Returns the velocity in RPM of the bottom shooter motor.
   * @return Velocity in RPM.
   */
  public double getVelocityBottom(){
    return shooterBottom.getVelocity().getValueAsDouble() * 60;
  }

  /**
   * Stops the motors.
   */
  public void stopMotors(){
    shooterTop.stopMotor();
    shooterBottom.stopMotor();
  }

  public boolean isAtShooterSpeed(ShooterSpeeds s){
    return Math.abs(getVelocityTop() - s.getRPMValue().getRPMValue()) < ShooterConstants.shooterSpeedThreshold && Math.abs(getVelocityBottom() - s.getRPMValue().getRPMValue()) < ShooterConstants.shooterSpeedThreshold;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Velocity(RPM) of the top motor", getVelocityTop());
    SmartDashboard.putNumber("Velocity(RPM) of the bottom motor", getVelocityBottom());

    // This method will be called once per scheduler run
    stateHandler.setBBFourCovered(getBeamBreakFour());
    // SmartDashboard.putNumber("Velocity of Primary Motor", shooterPrimary.getVelocity().getValueAsDouble());
    // SmartDashboard.putNumber("Velocity of Follower Motor", shooterFollower.getVelocity().getValueAsDouble());

    /*
     * 
     * 
     * if current speed != desired speed -> spin at desired speed
     */
  }
}