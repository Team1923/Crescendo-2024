// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateVariables.ArmPosition;
import frc.lib.RobotStateUtils.StateVariables.ArmStates;
import frc.lib.ShooterArmUtils.PositionRPMData;
import frc.lib.LimelightUtil.LimelightInterface;
import frc.lib.RobotStateUtils.StateHandler;



public class ArmSubsystem extends SubsystemBase {

  StateHandler stateHandler = StateHandler.getInstance();
  LimelightInterface limelightInterface = LimelightInterface.getInstance();
  PositionRPMData positionData = PositionRPMData.getInstance();
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 
    
  }
}