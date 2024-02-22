// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightUtil.LimelightInterface;
import frc.lib.RobotStateUtils.StateHandler;
import frc.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.Constants.LimeLightConstants;

public class LimelightSubsystem extends SubsystemBase {
  LimelightInterface limelight = LimelightInterface.getInstance();
  StateHandler stateHandler = StateHandler.getInstance();
  PositionRPMData rpmData = PositionRPMData.getInstance();
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  public double calculateDistanceToSpeakerTag(){
    double tagOffsetAngleVertical = limelight.getYAngleOffset();
    double angleToGoal = (LimeLightConstants.limelightMountAngle + tagOffsetAngleVertical) * (Math.PI / 180);
    return (LimeLightConstants.speakerHeightFromFloor - LimeLightConstants.limelightHeight) / Math.tan(angleToGoal);
  }

  @Override
  public void periodic() {
    stateHandler.setDistanceToSpeakerTag(calculateDistanceToSpeakerTag());
    stateHandler.setLimelightHasTag(limelight.hasValidTag());
    stateHandler.setAprilTagID(limelight.getID());
    stateHandler.setHasValidSpeakerTag(limelight.hasSpeakerTag());
    stateHandler.setHasValidAmpTag(limelight.hasAmpTag());
    stateHandler.setIsCenteredToTag(Math.abs(limelight.getXAngleOffset()) <= LimeLightConstants.xAngleThreshold && limelight.hasValidTag());

    // SmartDashboard.putNumber("Distance to April Tag", stateHandler.getDistanceToSpeakerTag());
    // SmartDashboard.putBoolean("Has Valid April Tag", stateHandler.getLimelightHasTag());
    // SmartDashboard.putNumber("April Tag ID", stateHandler.getAprilTagID());
    // SmartDashboard.putNumber("Predicted Angle of Arm", rpmData.getDesiredArmPosition(stateHandler.getDistanceToSpeakerTag()));
    // SmartDashboard.putNumber("Predicted RPM of the Tag", rpmData.getDesiredShooterRPM(stateHandler.getDistanceToSpeakerTag()));
    // //SmartDashboard.putBoolean("Has Valid Speaker April Tag", stateHandler.getHasValidSpeakerTag());
    // //SmartDashboard.putBoolean("Has Valid Amp April Tag", stateHandler.getHasValidAmpTag());
    // SmartDashboard.putNumber("X Angle Offset", limelight.getXAngleOffset());
    // SmartDashboard.putNumber("Y Angle Offset", limelight.getYAngleOffset());


  }


}