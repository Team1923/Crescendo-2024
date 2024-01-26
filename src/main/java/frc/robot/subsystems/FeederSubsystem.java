// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateVariables.FeederSpeeds;
import frc.robot.Constants.FeederConstants;
import frc.lib.RobotStateUtils.StateHandler;

public class FeederSubsystem extends SubsystemBase {
  // private TalonFX feederMotor = new TalonFX(FeederConstants.feederMotorID, "rio");

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    // feederMotor.getConfigurator().apply(new TalonFXConfiguration());
  }

  /**
   * This method sets the speed of the feeder motor wheel.
   * @param s The desired output speed.
   */
  public void setFeederMotorSpeed(double s) {
    // feederMotor.set(s);
  }

  /**
   * This method stops the motor.
   */
  public void stopFeederMotor() {
    // feederMotor.stopMotor();
  }

  @Override
  public void periodic() {
    
    FeederSpeeds desiredFeederSpeeds = StateHandler.getInstance().getDesiredFeederSpeed();
    switch(desiredFeederSpeeds) {
      case OFF:
        setFeederMotorSpeed(FeederSpeeds.OFF.percentOutputValue().getPercentOutputValue());
      case OUTWARD:
        setFeederMotorSpeed(FeederSpeeds.OUTWARD.percentOutputValue().getPercentOutputValue());
      case INWARD:
        setFeederMotorSpeed(FeederSpeeds.INWARD.percentOutputValue().getPercentOutputValue());
      default:
        stopFeederMotor();
    }

  }


  
}