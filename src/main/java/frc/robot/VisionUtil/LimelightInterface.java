package frc.robot.VisionUtil;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightInterface {
  // declare for easy calls
  private static LimelightInterface limelightInterface;

  // network table declarations
  private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  private double aprilTagID = 0;
  private boolean hasValidTarget = false;

  ArrayList<AprilTag> aprilTagList = new ArrayList<AprilTag>();
  public AprilTagFieldLayout aprilTagFieldLayout;

  public LimelightInterface() {
    fillAprilTagList();
    aprilTagFieldLayout = new AprilTagFieldLayout(aprilTagList, 16.54175, 8.0137);
  }

  /*
   * A singleton that creates the sole instance of BetterLimelightInterface,
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

  /*
   * the only setting we really need to do on the limelight
   * is the pipeline (if we decide to vision track)
   */
  public void setPipeline(int pipeline) {
    limelight.getEntry("pipeline").setNumber(pipeline);
  }

  public boolean hasValidTargets() {
    hasValidTarget = getDoubleEntry("tv") == 1.0;
    return hasValidTarget;
  }

  public double getTargetArea() {
    return getDoubleEntry("ta");
  }

  public double getXOffset() {
    return getDoubleEntry("tx");
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

  public double getID() {
    aprilTagID = getDoubleEntry("tid");
    return aprilTagID;
  }

  public boolean hasScoringTarget() {
    return (((getID() == 1) || (getID() == 2) || (getID() == 3)
        || (getID() == 6) || (getID() == 7) || (getID() == 8)) && hasValidTargets());
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
    if (hasValidTargets()) {
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


    }
}