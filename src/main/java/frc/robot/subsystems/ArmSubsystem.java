// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateVariables.ArmStates;
import frc.lib.RobotStateUtils.StateHandler;


public class ArmSubsystem extends SubsystemBase {

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ArmStates desiredArmState = StateHandler.getInstance().getDesiredArmState();

    switch(desiredArmState) {
      case STOWED:
        //setPosition(ArmStates.HOME.getArmPosition().getAngularSetpoint());
      case AMP:
        //setPosition(ArmStates.AMP_EJECT.getArmPosition().getAngularSetpoint());
      case SPEAKER:
        //setPosition(positionData.getDesiredArmPosition(LimelightInterface.getInstance().getXOffset()));
      default:
        //idk
    }
  }
}