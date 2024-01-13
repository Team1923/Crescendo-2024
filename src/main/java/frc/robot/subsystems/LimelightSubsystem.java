// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.RobotStateUtils.StateHandler;
import frc.robot.VisionUtil.LimelightInterface;

public class LimelightSubsystem extends SubsystemBase {
  LimelightInterface limelight = LimelightInterface.getInstance();
  StateHandler stateHandler = StateHandler.getInstance();
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  public double calculateDistanceToTag(){
    double tagOffsetAngleVertical = limelight.getYAngleOffset();
    double angleToGoal = (LimeLightConstants.limelightMountAngle + tagOffsetAngleVertical) * (Math.PI/ 180);
    return (LimeLightConstants.speakerHeightFromFloor - LimeLightConstants.limelightHeight) / Math.tan(angleToGoal);
  }

  @Override
  public void periodic() {
    stateHandler.setDistanceToTag(calculateDistanceToTag());
    stateHandler.setLimelightHasTag(limelight.hasValidTag());
    stateHandler.setAprilTagID(limelight.getID());
    stateHandler.setHasValidSpeakerTag(limelight.hasSpeakerTag());
    stateHandler.setHasValidAmpTag(limelight.hasAmpTag());

    SmartDashboard.putNumber("Distance to April Tag", stateHandler.getDistanceToTag());
    SmartDashboard.putBoolean("Has Valid April Tag", stateHandler.getLimelightHasTag());
    SmartDashboard.putNumber("April Tag ID", stateHandler.getAprilTagID());
    SmartDashboard.putBoolean("Has Valid Speaker April Tag", stateHandler.getHasValidSpeakerTag());
    SmartDashboard.putBoolean("Has Valid Amp April Tag", stateHandler.getHasValidAmpTag());

  }


}
