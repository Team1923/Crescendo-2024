package frc.robot.subsystems;

import org.opencv.core.Point;

import com.ctre.phoenix6.Utils;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.lib.Limelight.LimelightInterface;
import frc.robot.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.lib.SimUtils.SimulationSubsystem;
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
  public double calculateDistanceToCoveredSpeakerTag() {
    Point robotPos = new Point(stateHandler.getRobotPose().getX(), stateHandler.getRobotPose().getY());
    Point blueSpeakerPos = FieldConstants.blueSpeakerPos;
    Point redSpeakerPos = FieldConstants.redSpeakerPos;
    Point deltaPos;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      deltaPos = new Point( robotPos.x - blueSpeakerPos.x ,  robotPos.y - blueSpeakerPos.y);
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

    if (Utils.isSimulation()){
      
      stateHandler.setxAngleOffset(SimulationSubsystem.getInstance().simLLAngle() == 100 ? 0 : SimulationSubsystem.getInstance().simLLAngle());
      SmartDashboard.putNumber("Sim ll offset", stateHandler.getxAngleOffset());
    }
    else{
      stateHandler.setxAngleOffset(limelight.getXAngleOffset());

    }

    if (Utils.isSimulation()){
      stateHandler.setDistanceToSpeakerTag(calculateDistanceToCoveredSpeakerTag());
    }
    /* NOTE: inAutoOverride AND having a valid tag = optimized auto */
    else if (DriverStation.isAutonomousEnabled() && !limelight.hasValidTag()) {
      stateHandler.setCoveredSpeakerTagDistance(calculateDistanceToCoveredSpeakerTag());
    } else {
      stateHandler.setDistanceToSpeakerTag(calculateDistanceToSpeakerTag());
    }
    
    stateHandler.setDistanceToTrapTag(calculateDistanceToTrapTag());
    stateHandler.setLimelightHasTag(limelight.hasValidTag());
    stateHandler.setAprilTagID(limelight.getID());

    if (Utils.isSimulation()){
      stateHandler.setHasValidSpeakerTag(SimulationSubsystem.getInstance().simLLAngle() != 100);
    }
    else{
        stateHandler.setHasValidSpeakerTag(limelight.hasSpeakerTag());

    }

    
    stateHandler.setHasValidAmpTag(limelight.hasAmpTag());
    stateHandler.setHasValidTrapTag(limelight.hasTrapTag());

    if (Utils.isSimulation()){
      stateHandler.setIsCenteredToTag(stateHandler.getxAngleOffset()!= 100 && Math.abs(stateHandler.getxAngleOffset()) <= LimeLightConstants.xAngleThreshold+1);
    }
    else{
      stateHandler.setIsCenteredToTag(!stateHandler.getAutoOverride() ? Math.abs(limelight.getXAngleOffset()) <= LimeLightConstants.xAngleThreshold && limelight.hasValidTag() : true);

    }

    

    SmartDashboard.putNumber("Distance to Speaker April Tag", stateHandler.getDistanceToSpeakerTag());
    SmartDashboard.putNumber("Distance to Trap April Tag", stateHandler.getDistanceToTrapTag());

    SmartDashboard.putNumber("Covered Distance", this.calculateDistanceToCoveredSpeakerTag());

    // SmartDashboard.putBoolean("Has Valid April Tag", stateHandler.getLimelightHasTag());
    // SmartDashboard.putNumber("April Tag ID", stateHandler.getAprilTagID());
    // SmartDashboard.putNumber("Predicted Angle of Arm", rpmData.getDesiredArmPosition(stateHandler.getDistanceToSpeakerTag()));
    // SmartDashboard.putNumber("Predicted RPM of the Tag", rpmData.getDesiredShooterRPM(stateHandler.getDistanceToSpeakerTag()));
    // //SmartDashboard.putBoolean("Has Valid Speaker April Tag", stateHandler.getHasValidSpeakerTag());
    // //SmartDashboard.putBoolean("Has Valid Amp April Tag", stateHandler.getHasValidAmpTag());
    SmartDashboard.putNumber("X Angle Offset", limelight.getXAngleOffset());
    // SmartDashboard.putNumber("Y Angle Offset", limelight.getYAngleOffset());
      SmartDashboard.putBoolean("Centered to tag", stateHandler.getIsCenteredToTag());

  }


}
