// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.SimUtils;

import java.util.ArrayList;
import java.util.TreeSet;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.StateMachine.StateHandler;

public class SimulationSubsystem extends SubsystemBase {

  private static final double collectionDist = 0.5; //meters

  StateHandler stateHandler = StateHandler.getInstance();

  private class IntakeTimes{
    private static final double bb1OffTime = 0.1;
    private static final double bb2OnTime = 0.08;
    private static final double bb3OnTime = 0.2;
  }



  private ArrayList<Translation2d> notePoses;

  private boolean isCollecting = false;
  private Timer collectionTimer;




  private Pose2d currentPose;

  // public static synchronized SimulationSubsystem getInstance() {
  //       if (simSub == null) {
  //           simSub = new SimulationSubsystem();
  //       }

  //       return simSub;
  //   }


  /** Creates a new SimulationSubsystem. */
  public SimulationSubsystem() {
    notePoses = new ArrayList<>();

    notePoses.add(new Translation2d(2.89, 4.10)); //podiumLoc
    notePoses.add(new Translation2d(2.89, 5.56)); //midLoc
    notePoses.add(new Translation2d(2.89, 7.00)); //ampLoc

    notePoses.add(new Translation2d(8.28, 7.45)); //1Loc
    notePoses.add(new Translation2d(8.28, 5.78)); //2Loc
    notePoses.add(new Translation2d(8.28, 4.09)); //3Loc
    notePoses.add(new Translation2d(8.28, 2.44)); //4Loc
    notePoses.add(new Translation2d(8.28,0.76)); //5Loc


   currentPose = new Pose2d();

   collectionTimer = new Timer();

  }

  public void updatePose(Pose2d robotPose){
    currentPose = robotPose;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (isCollecting){
      if (collectionTimer.get()==0){
        collectionTimer.start();
        stateHandler.setBBOneCovered(true);
      }

      if (collectionTimer.get()>=IntakeTimes.bb1OffTime){
        stateHandler.setBBOneCovered(false);
      }

      if (collectionTimer.get()>=IntakeTimes.bb2OnTime){
        stateHandler.setBBTwoCovered(true);
      }

      if (collectionTimer.get()>IntakeTimes.bb3OnTime){
        stateHandler.setBBThreeCovered(true);

        isCollecting = false;
        collectionTimer.stop();
        collectionTimer.reset();
      }

    }
    else{
      for (int i = notePoses.size()-1; i >=0; i--){
        Translation2d notePos = notePoses.get(i);
        if (notePos != null && notePos.getDistance(currentPose.getTranslation())< collectionDist){

          isCollecting = true;

          notePoses.remove(i);
        }
      }
    }

    SmartDashboard.putNumber("collecting timer", collectionTimer.get());

    SmartDashboard.putBoolean("isCollecting", isCollecting);

    SmartDashboard.putString("sim pose", currentPose.toString());

  }
}
