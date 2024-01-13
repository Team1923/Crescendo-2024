package frc.robot.VisionUtil;

import java.util.ArrayList;
import java.util.Optional;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightInterface {
  // declare for easy calls
  private static LimelightInterface limelightInterface;

  // network table declarations
  private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");


  ArrayList<AprilTag> aprilTagList = new ArrayList<AprilTag>();
  public AprilTagFieldLayout aprilTagFieldLayout;

  public LimelightInterface() {
    fillAprilTagList();
    aprilTagFieldLayout = new AprilTagFieldLayout(aprilTagList, 16.451, 8.211);
  }

  /*
   * A singleton that creates the sole instance of LimelightInterface,
   * this is also used in other classes as well, so we don't have to pass in
   * new instances all the time
   */
  public static synchronized LimelightInterface getInstance() {
    if (limelightInterface == null) {
      limelightInterface = new LimelightInterface();
    }
    return limelightInterface;
  }

  public ArrayList<AprilTag> getAprilTagList() {
    return aprilTagList;
  }

  public double getDoubleEntry(String entry) {
    return limelight.getEntry(entry).getDouble(0);
  }

  public double[] getArrayEntry(String entry) {
    return limelight.getEntry(entry).getDoubleArray(new double[6]);
  }


  public boolean hasValidTag() {
    return getDoubleEntry("tv") == 1.0;
  }

  public double getTagArea() {
    return getDoubleEntry("ta");
  }

  /**
   * The X Offset from the camera to the tag in DEGREES.
   * @return The Angle offset from the tag. 
   */
  public double getXAngleOffset() {
    return getDoubleEntry("tx");
  }

  /**
   * The Y Offset from the camera to the tag in DEGREES.
   * @return The Angle offset from the tag.
   */
  public double getYAngleOffset() {
    return getDoubleEntry("ty");
  }

  /*
   * Specific AprilTag methods we will need
   */

  /*
   * This method returns the pose of the robot, a combination of translational
   * and rotational offset relative to the april tag
   */
  public double[] getBotPose() {
    return getArrayEntry("botpose_targetspace");
  }

  public double getID(){
    return hasValidTag() ? getDoubleEntry("tid") : -1;
  }

  public boolean hasSpeakerTag() { 
    return (((getID() == 3) || (getID() == 4) || (getID() == 7) || (getID() == 8)) && hasValidTag());
  }

  public boolean hasAmpTag(){
    return(((getID() == 6)|| (getID() == 5)) && hasValidTag());
  }

  public double getTL() {
    return getDoubleEntry("tl");
  }

  public double getCL() {
    return getDoubleEntry("cl");
  }

  /*
   * create a Pose3D object for trajectory generation
   */
  public Pose3d getRobotPose3d() {
    double[] result = getBotPose();
    Translation3d tran3d = new Translation3d(result[0], result[1], result[2]);
    Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);
    Pose3d p3d = new Pose3d(tran3d, r3d);

    return p3d;
  }

  public Pose3d getAprilTagPose() {
    if (hasValidTag()) {
      Optional<Pose3d> aprilTagPose = aprilTagFieldLayout.getTagPose((int) getID());
      if (aprilTagPose.isEmpty()) {
        return new Pose3d();
      }
      return aprilTagPose.get();
    }
    return new Pose3d();
  }

  public Pose3d getAprilTagPose(int id) {
      return aprilTagFieldLayout.getTagPose(id).get();
  }

  /* Change to be season-specifc */
  public void fillAprilTagList() {
    aprilTagList
      .add(new AprilTag(1, new Pose3d(15.079471999999997, 0.24587199999999998, 1.355852, new Rotation3d(new Quaternion(0.500000000000000, 0, 0, 0.8660254037844386)))));
    aprilTagList
      .add(new AprilTag(2, new Pose3d(16.185134, 0.883666,1.355852, new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386)))));
    aprilTagList
      .add(new AprilTag(3, new Pose3d(16.579342, 4.982717999999999,1.4511020000000001, new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1)))));
    aprilTagList
      .add(new AprilTag(4, new Pose3d(16.579342, 5.547867999999999,1.4511020000000001, new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1)))));
    aprilTagList
      .add(new AprilTag(5, new Pose3d(14.700757999999999, 8.2042,1.355852, new Rotation3d(new Quaternion(-0.7071067811865475, -0.0, 0, 0.7071067811865476)))));
    aprilTagList
      .add(new AprilTag(6, new Pose3d(1.8415, 8.2042,1.355852, new Rotation3d(new Quaternion(-0.7071067811865475, -0.0, 0, 0.7071067811865476)))));
    aprilTagList
      .add(new AprilTag(7, new Pose3d(-0.038099999999999995, 5.547867999999999,1.4511020000000001, new Rotation3d(new Quaternion(1.0, 0.0, 0, 0.0)))));
    aprilTagList
      .add(new AprilTag(8, new Pose3d(-0.038099999999999995, 4.982717999999999,1.4511020000000001, new Rotation3d(new Quaternion(1.0, 0.0, 0, 0.0)))));
    aprilTagList
      .add(new AprilTag(9, new Pose3d(0.356108, 0.883666,1.355852, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0, 0.49999999999999994)))));
    aprilTagList
      .add(new AprilTag(10, new Pose3d(1.4615159999999998, 0.24587199999999998,1.355852, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0, 0.49999999999999994))))); 
    aprilTagList
      .add(new AprilTag(10, new Pose3d(1.4615159999999998, 0.24587199999999998,1.355852, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0, 0.49999999999999994)))));
    aprilTagList
      .add(new AprilTag(11, new Pose3d(11.904726, 3.7132259999999997,1.3208, new Rotation3d(new Quaternion(-0.8660254037844387, 0.0, 0, 0.49999999999999994)))));
    aprilTagList
      .add(new AprilTag(12, new Pose3d(11.904726, 4.49834,1.3208, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0, 0.49999999999999994)))));
    aprilTagList
      .add(new AprilTag(13, new Pose3d(11.220196, 4.105148,1.3208, new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0, 1.0)))));
    aprilTagList
      .add(new AprilTag(14, new Pose3d(5.320792, 4.105148,1.3208, new Rotation3d(new Quaternion(1, 0.0, 0, 0)))));
    aprilTagList
      .add(new AprilTag(15, new Pose3d(4.641342, 4.49834,1.3208, new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0, 0.8660254037844386)))));
    aprilTagList
      .add(new AprilTag(16, new Pose3d(4.641342, 3.7132259999999997,1.3208, new Rotation3d(new Quaternion(-0.4999999999999998, 0.0, 0, 0.8660254037844387)))));
    }
}