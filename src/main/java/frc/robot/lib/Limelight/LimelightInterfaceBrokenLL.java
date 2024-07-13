package frc.robot.lib.Limelight;

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

public class LimelightInterfaceBrokenLL extends LimelightInterface {
  // declare for easy calls
  private static LimelightInterface limelightInterface;

  // network table declarations
  // private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-shooter");


  ArrayList<AprilTag> aprilTagList = new ArrayList<AprilTag>();
  public AprilTagFieldLayout aprilTagFieldLayout;

  public LimelightInterfaceBrokenLL() {
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

  @Override
  public double getDoubleEntry(String entry) {
    return 0;
  }
  @Override
  public double[] getArrayEntry(String entry) {
    return new double[1];
  }


}