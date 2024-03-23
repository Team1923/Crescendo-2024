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
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
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


      stateHandler.setCurrentTagPose(limelight.getAprilTagPose());

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

      SmartDashboard.putNumber("calc trap heading", getSeenTrapHeading());
      

  }


  public static double getSeenTrapHeading(){
    //trap tags are 11 through 16
    if (!(StateHandler.getInstance().getAprilTagID()>=11)){
      return -1;
    }

    Pose3d currentTagPose = StateHandler.getInstance().getCurrentTagPose();



    Rotation3d tagRotation = currentTagPose.getRotation();

    double rawHeading = Math.toDegrees(tagRotation.getZ());

    // System.out.println(rawHeading);

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      return Math.IEEEremainder(180+rawHeading, 360);
    }
    else{


      return Math.IEEEremainder(rawHeading, 360);
    }
    

  }

  // public static double getSeenTrapHeading(int id, Alliance a){

  //   ArrayList<AprilTag> aprilTagList = new ArrayList<>();

  //   aprilTagList
  //     .add(new AprilTag(1, new Pose3d(15.079471999999997, 0.24587199999999998, 1.355852, new Rotation3d(new Quaternion(0.500000000000000, 0, 0, 0.8660254037844386)))));
  //   aprilTagList
  //     .add(new AprilTag(2, new Pose3d(16.185134, 0.883666,1.355852, new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386)))));
  //   aprilTagList
  //     .add(new AprilTag(3, new Pose3d(16.579342, 4.982717999999999,1.4511020000000001, new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1)))));
  //   aprilTagList
  //     .add(new AprilTag(4, new Pose3d(16.579342, 5.547867999999999,1.4511020000000001, new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1)))));
  //   aprilTagList
  //     .add(new AprilTag(5, new Pose3d(14.700757999999999, 8.2042,1.355852, new Rotation3d(new Quaternion(-0.7071067811865475, -0.0, 0, 0.7071067811865476)))));
  //   aprilTagList
  //     .add(new AprilTag(6, new Pose3d(1.8415, 8.2042,1.355852, new Rotation3d(new Quaternion(-0.7071067811865475, -0.0, 0, 0.7071067811865476)))));
  //   aprilTagList
  //     .add(new AprilTag(7, new Pose3d(-0.038099999999999995, 5.547867999999999,1.4511020000000001, new Rotation3d(new Quaternion(1.0, 0.0, 0, 0.0)))));
  //   aprilTagList
  //     .add(new AprilTag(8, new Pose3d(-0.038099999999999995, 4.982717999999999,1.4511020000000001, new Rotation3d(new Quaternion(1.0, 0.0, 0, 0.0)))));
  //   aprilTagList
  //     .add(new AprilTag(9, new Pose3d(0.356108, 0.883666,1.355852, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0, 0.49999999999999994)))));
  //   aprilTagList
  //     .add(new AprilTag(10, new Pose3d(1.4615159999999998, 0.24587199999999998,1.355852, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0, 0.49999999999999994))))); 
  //   aprilTagList
  //     .add(new AprilTag(10, new Pose3d(1.4615159999999998, 0.24587199999999998,1.355852, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0, 0.49999999999999994)))));
  //   aprilTagList
  //     .add(new AprilTag(11, new Pose3d(11.904726, 3.7132259999999997,1.3208, new Rotation3d(new Quaternion(-0.8660254037844387, 0.0, 0, 0.49999999999999994)))));
  //   aprilTagList
  //     .add(new AprilTag(12, new Pose3d(11.904726, 4.49834,1.3208, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0, 0.49999999999999994)))));
  //   aprilTagList
  //     .add(new AprilTag(13, new Pose3d(11.220196, 4.105148,1.3208, new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0, 1.0)))));
  //   aprilTagList
  //     .add(new AprilTag(14, new Pose3d(5.320792, 4.105148,1.3208, new Rotation3d(new Quaternion(1, 0.0, 0, 0)))));
  //   aprilTagList
  //     .add(new AprilTag(15, new Pose3d(4.641342, 4.49834,1.3208, new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0, 0.8660254037844386)))));
  //   aprilTagList
  //     .add(new AprilTag(16, new Pose3d(4.641342, 3.7132259999999997,1.3208, new Rotation3d(new Quaternion(-0.4999999999999998, 0.0, 0, 0.8660254037844387)))));
    



  //   //trap tags are 11 through 16
  //   if (!(id>=11)){
  //     return -1;
  //   }

  //   AprilTag currentTag = aprilTagList.get(id);

  //   System.out.println("ID: " + currentTag.ID);


  //   Pose3d currentTagPose = currentTag.pose;



  //   Rotation3d tagRotation = currentTagPose.getRotation();

  //   double rawHeading = Math.toDegrees(tagRotation.getZ());

  //   System.out.println(rawHeading);

  //   if (a == DriverStation.Alliance.Red){
  //     return Math.IEEEremainder(180+rawHeading, 360);
  //   }
  //   else{


  //     return Math.IEEEremainder(rawHeading, 360);
  //   }
    

  // }

  // public static void main(String[] args){
  //   System.out.println(getSeenTrapHeading(16, Alliance.Blue));
  // }




}
