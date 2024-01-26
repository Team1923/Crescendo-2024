// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateHandler;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private DigitalInput beamBreakOne = new DigitalInput(IntakeConstants.beamBreakOneID);
  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  
  }

  /**
   * Method to get the digital input reading of BB1.
   * TODO: verify if adding a NOT before the boolean is needed.
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakOne() {
    return beamBreakOne.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    stateHandler.setBBOneCovered(getBeamBreakOne());
  }
}