// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateUtils.StateHandler;
import frc.robot.RobotStateUtils.StateVariables.ArmStates;
import frc.robot.ShooterArmDataUtils.PositionRPMData;
import frc.robot.VisionUtil.LimelightInterface;

public class ArmSubsystem extends SubsystemBase {

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ArmStates desiredArmState = StateHandler.getInstance().getDesiredArmState();

    switch(desiredArmState) {
      case HOME:
        //setPosition(ArmStates.HOME.getArmPosition().getAngularSetpoint());
      case AMP_EJECT:
        //setPosition(ArmStates.AMP_EJECT.getArmPosition().getAngularSetpoint());
      case SHOOTING:
        //setPosition(positionData.getDesiredArmPosition(LimelightInterface.getInstance().getXOffset()));
      default:
        //idk
    }
  }
}
