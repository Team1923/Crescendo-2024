package frc.robot.subsystems;

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
  public double calculateDistanceToSpeakerTag(){
    double tagOffsetAngleVertical = limelight.getYAngleOffset();
    double angleToGoal = (LimeLightConstants.limelightMountAngle + tagOffsetAngleVertical) * (Math.PI / 180);
    return (LimeLightConstants.speakerHeightFromFloor - LimeLightConstants.limelightHeight) / Math.tan(angleToGoal);
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
    //Update Limelight data 
    stateHandler.setDistanceToSpeakerTag(calculateDistanceToSpeakerTag());
    stateHandler.setDistanceToTrapTag(calculateDistanceToTrapTag());
    stateHandler.setLimelightHasTag(limelight.hasValidTag());
    stateHandler.setAprilTagID(limelight.getID());
    stateHandler.setHasValidSpeakerTag(limelight.hasSpeakerTag());
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
