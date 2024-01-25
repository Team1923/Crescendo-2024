// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  // private TalonFX shooterPrimary = new TalonFX(Constants.ShooterConstants.shooterMotorPrimaryID);
  // private TalonFX shooterFollower = new TalonFX(Constants.ShooterConstants.shooterMotorFollowerID);

  private VelocityVoltage velocityPrimary = new VelocityVoltage(0);
  private VelocityVoltage velocityFollower = new VelocityVoltage(0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // shooterPrimary.getConfigurator().apply(new TalonFXConfiguration());
    // shooterFollower.getConfigurator().apply(new TalonFXConfiguration());

    // shooterPrimary.setInverted(true);

    var slot0Configs = new Slot0Configs();
    /**
     * Rough values, need to tune them to final robot.
     */
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11; 
    slot0Configs.kI = 0.48; 
    slot0Configs.kD = 0.01; 

    // shooterPrimary.getConfigurator().apply(slot0Configs, 0.05);
    // shooterFollower.getConfigurator().apply(slot0Configs, 0.05);
  }

  /**
   * Method to set speed of shooter motors, using velocity closed loop control.
   * @param velocity The speed, in RPM, passed into the motors.
   */
  public void set(double velocityP, double velocityF){
    velocityPrimary.Slot = 0;
    velocityFollower.Slot = 0;
    // shooterPrimary.setControl(velocityPrimary.withVelocity(velocityP));//change
    // shooterFollower.setControl(velocityFollower.withVelocity(velocityF));//change
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Velocity of Primary Motor", shooterPrimary.getVelocity().getValueAsDouble());
    // SmartDashboard.putNumber("Velocity of Follower Motor", shooterFollower.getVelocity().getValueAsDouble());
  }
}