package frc.robot.subsystems;

import java.sql.Driver;
import java.util.ArrayList;

import org.opencv.core.Point;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
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

      SimulationSubsystem sim = SimulationSubsystem.getInstance();

      stateHandler.setAprilTagID(sim.getSeenTagID());

      if (stateHandler.getAprilTagID() != -1){

        stateHandler.setLimelightHasTag(true);

          Pose3d currentTag = LimelightInterface.getInstance().getAprilTagPose(stateHandler.getAprilTagID());
          stateHandler.setxAngleOffset(sim.simLLAngleToPoint(currentTag));

          if (stateHandler.getAprilTagID() == 4 || stateHandler.getAprilTagID() == 7){
            stateHandler.setHasValidSpeakerTag(true);
            stateHandler.setDistanceToSpeakerTag(sim.getDistFromRobotToPose(currentTag));

            stateHandler.setHasValidAmpTag(false);
            stateHandler.setHasValidTrapTag(false);
          }
          else if (stateHandler.getAprilTagID()>=11){
            stateHandler.setHasValidTrapTag(true);
            stateHandler.setDistanceToTrapTag(sim.getDistFromRobotToPose(currentTag));

            stateHandler.setHasValidSpeakerTag(false);
            stateHandler.setHasValidTrapTag(false);
          }
          else if (stateHandler.getAprilTagID() == 5 || stateHandler.getAprilTagID() == 6){

            stateHandler.setHasValidAmpTag(true);


            stateHandler.setHasValidSpeakerTag(false);
            stateHandler.setHasValidTrapTag(false);
          }

          stateHandler.setCurrentTagPose(currentTag);


      }
      else{

        stateHandler.setAprilTagID(-1);

        stateHandler.setLimelightHasTag(false);
        stateHandler.setHasValidAmpTag(false);
        stateHandler.setHasValidSpeakerTag(false);
        stateHandler.setHasValidTrapTag(false);
        stateHandler.setIsCenteredToTag(false);

        stateHandler.setCurrentTagPose(null);

      }




      stateHandler.setIsCenteredToTag(stateHandler.getxAngleOffset()!= 100 && Math.abs(stateHandler.getxAngleOffset()) <= LimeLightConstants.xAngleThreshold+1);



    }
    else{
      stateHandler.setLimelightHasTag(limelight.hasValidTag());

      stateHandler.setAprilTagID((int)limelight.getID());
      stateHandler.setxAngleOffset(limelight.getXAngleOffset());

      /* NOTE: inAutoOverride AND having a valid tag = optimized auto */
      if (DriverStation.isAutonomousEnabled() && !limelight.hasValidTag()) {
      stateHandler.setCoveredSpeakerTagDistance(calculateDistanceToCoveredSpeakerTag());
      }
      else {
      stateHandler.setDistanceToSpeakerTag(calculateDistanceToSpeakerTag());
      }

      stateHandler.setDistanceToTrapTag(calculateDistanceToTrapTag());

      stateHandler.setHasValidSpeakerTag(limelight.hasSpeakerTag());


      stateHandler.setHasValidAmpTag(limelight.hasAmpTag());
      stateHandler.setHasValidTrapTag(limelight.hasTrapTag());

      stateHandler.setIsCenteredToTag(!stateHandler.getAutoOverride() ? Math.abs(limelight.getXAngleOffset()) <= LimeLightConstants.xAngleThreshold && limelight.hasValidTag() : true);


      stateHandler.setCurrentTagPose(LimelightInterface.getInstance().getAprilTagPose());

    }



   // SmartDashboard.putNumber("Distance to Speaker April Tag", stateHandler.getDistanceToSpeakerTag());
   // SmartDashboard.putNumber("Distance to Trap April Tag", stateHandler.getDistanceToTrapTag());

    SmartDashboard.putNumber("Covered Distance", this.calculateDistanceToCoveredSpeakerTag());

    //SmartDashboard.putBoolean("Has Valid April Tag", stateHandler.getLimelightHasTag());
    //SmartDashboard.putNumber("April Tag ID", stateHandler.getAprilTagID());
    // SmartDashboard.putNumber("Predicted Angle of Arm", rpmData.getDesiredArmPosition(stateHandler.getDistanceToSpeakerTag()));
    // SmartDashboard.putNumber("Predicted RPM of the Tag", rpmData.getDesiredShooterRPM(stateHandler.getDistanceToSpeakerTag()));
    //SmartDashboard.putBoolean("Has Valid Speaker April Tag", stateHandler.getHasValidSpeakerTag());
    //SmartDashboard.putBoolean("Has Valid Amp April Tag", stateHandler.getHasValidAmpTag());
    //SmartDashboard.putNumber("X Angle Offset", stateHandler.getxAngleOffset());
    // SmartDashboard.putNumber("Y Angle Offset", limelight.getYAngleOffset());
      SmartDashboard.putBoolean("Centered to tag", stateHandler.getIsCenteredToTag());

  }


  public static double getSeenTrapHeading(){

    //trap tags are 11 through 16
    if (!(StateHandler.getInstance().getAprilTagID()>=11)){
      return -1;
    }

    Pose3d currentTagPose = StateHandler.getInstance().getCurrentTagPose();



    Rotation3d tagRotation = currentTagPose.getRotation();

    double rawHeading = Math.toDegrees(tagRotation.getZ());

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      return rawHeading;
    }
    else{

      double calcedHeading = 180 + rawHeading;

      if (calcedHeading>180){
        calcedHeading = calcedHeading-360;
      }

      return 180 - calcedHeading;
    }
    

  }




}
