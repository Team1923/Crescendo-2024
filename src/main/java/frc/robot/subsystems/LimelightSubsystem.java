package frc.robot.subsystems;

import org.opencv.core.Point;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.lib.Limelight.LimelightInterface;
import frc.robot.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.lib.StateMachine.StateHandler;

public class LimelightSubsystem extends SubsystemBase {
  LimelightInterface limelight = LimelightInterface.getInstance();
  StateHandler stateHandler = StateHandler.getInstance();
  PositionRPMData rpmData = PositionRPMData.getInstance();
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  /**
   * This method returns the distance to the closest speaker tag using math provided by Limelight docs.
   * @return The distance from limelight to tag in inches.
   */
  public double calculateDistanceToSpeakerTag() {
    double tagOffsetAngleVertical = limelight.getYAngleOffset();
    double angleToGoal = (LimeLightConstants.limelightMountAngle + tagOffsetAngleVertical) * (Math.PI / 180);
    return (LimeLightConstants.speakerHeightFromFloor - LimeLightConstants.limelightHeight) / Math.tan(angleToGoal);
  }

  /**
   * Returns the PP estimated distance based on swerve pose estimation.
   * @return Distance from robot to speaker in inches.
   */
  public double calculateDistanceToCoveredTag() {
    Point robotPos = new Point(stateHandler.getRobotPose().getX(), stateHandler.getRobotPose().getY());
    Point blueSpeakerPos = new Point(0.9, 5.5);
    Point redSpeakerPos = new Point(15.6, 5.6);
    Point deltaPos;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      deltaPos = new Point(robotPos.x - blueSpeakerPos.x, robotPos.y - blueSpeakerPos.y);
    } else {
      deltaPos = new Point(redSpeakerPos.x - robotPos.x, redSpeakerPos.y - robotPos.y);
    }

    return Units.metersToInches(Math.hypot(deltaPos.x, deltaPos.y));
  }

  /**
   * This method returns the distance to the closest trap tag using math provided by Limelight docs.
   * @return The distance from limelight to tag in inches.
   */
  public double calculateDistanceToTrapTag() {
    double tagOffsetAngleVertical = limelight.getYAngleOffset();
    double angleToGoal = (LimeLightConstants.limelightMountAngle + tagOffsetAngleVertical) * (Math.PI / 180);
    return (LimeLightConstants.trapHeightFromFloor - LimeLightConstants.limelightHeight) / Math.tan(angleToGoal);
  }

  @Override
  public void periodic() {
    /* NOTE: inAutoOverride AND having a valid tag = optimized auto */
    if (stateHandler.getAutoOverride() && limelight.hasValidTag()) {
      stateHandler.setDistanceToSpeakerTag(calculateDistanceToSpeakerTag());
    } else if (stateHandler.getAutoOverride() && !limelight.hasValidTag()) {
      stateHandler.setDistanceToSpeakerTag(calculateDistanceToCoveredTag());
    } else {
      stateHandler.setDistanceToSpeakerTag(calculateDistanceToSpeakerTag());
    }
    
    stateHandler.setDistanceToTrapTag(calculateDistanceToTrapTag());
    stateHandler.setLimelightHasTag(limelight.hasValidTag());
    stateHandler.setAprilTagID(limelight.getID());
    if (!stateHandler.getAutoOverride()) {
      stateHandler.setHasValidSpeakerTag(limelight.hasSpeakerTag());
    } else {
      stateHandler.setHasValidSpeakerTag(true);
    }
    
    stateHandler.setHasValidAmpTag(limelight.hasAmpTag());
    stateHandler.setHasValidTrapTag(limelight.hasTrapTag());
    stateHandler.setIsCenteredToTag(Math.abs(limelight.getXAngleOffset()) <= LimeLightConstants.xAngleThreshold && limelight.hasValidTag());

    SmartDashboard.putNumber("Distance to Speaker April Tag", stateHandler.getDistanceToSpeakerTag());
    SmartDashboard.putNumber("Distance to Trap April Tag", stateHandler.getDistanceToTrapTag());

    // SmartDashboard.putBoolean("Has Valid April Tag", stateHandler.getLimelightHasTag());
    // SmartDashboard.putNumber("April Tag ID", stateHandler.getAprilTagID());
    // SmartDashboard.putNumber("Predicted Angle of Arm", rpmData.getDesiredArmPosition(stateHandler.getDistanceToSpeakerTag()));
    // SmartDashboard.putNumber("Predicted RPM of the Tag", rpmData.getDesiredShooterRPM(stateHandler.getDistanceToSpeakerTag()));
    // //SmartDashboard.putBoolean("Has Valid Speaker April Tag", stateHandler.getHasValidSpeakerTag());
    // //SmartDashboard.putBoolean("Has Valid Amp April Tag", stateHandler.getHasValidAmpTag());
    SmartDashboard.putNumber("X Angle Offset", limelight.getXAngleOffset());
    // SmartDashboard.putNumber("Y Angle Offset", limelight.getYAngleOffset());


  }


}
