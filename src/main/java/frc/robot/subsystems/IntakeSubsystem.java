// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * FINISH
 */

public class IntakeSubsystem extends SubsystemBase {

  // private TalonFX intakePrimaryMotor = new TalonFX(IntakeConstants.intakePrimaryID, "rio");
  // private TalonFX intakeFollowerMotor = new TalonFX(IntakeConstants.intakeFollowerID, "rio");
  // private TalonFX intakeRollerMotor = new TalonFX(IntakeConstants.intakeRollerMotorID, "rio");

  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // intakePrimaryMotor.getConfigurator().apply(new TalonFXConfiguration());
    // intakeFollowerMotor.getConfigurator().apply(new TalonFXConfiguration());
    // intakeRollerMotor.getConfigurator().apply(new TalonFXConfiguration());

  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
