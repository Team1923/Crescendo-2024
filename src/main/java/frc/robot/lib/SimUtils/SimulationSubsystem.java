// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.SimUtils;

import java.util.ArrayList;
import java.util.TreeSet;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.IntakeRollerSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeStates;

public class SimulationSubsystem extends SubsystemBase {

  private static SimulationSubsystem simSub = new SimulationSubsystem();


  private static final double collectionDist = 0.75; //meters



  StateHandler stateHandler = StateHandler.getInstance();

  public static synchronized SimulationSubsystem getInstance(){
      if (simSub == null){
        simSub = new SimulationSubsystem();
      }
      return simSub;
  }

  private class IntakeTimes{
    private static final double bb1OffTime = 0.1;
    private static final double bb2OnTime = 0.08;
    private static final double bb3OnTime = 0.2;
  }

  private class ShootTimes{
    private static final double bb2OffTime = 0.04;
    private static final double bb3OffTime = 0.2;
    private static final double bb4OffTime = 0.3;
  }




  private ArrayList<Translation2d> notePoses;

  private boolean isCollecting = false;
  private Timer collectionTimer;

  private boolean isShooting = false;
  private Timer shootTimer;




  private Pose2d currentPose;

  // public static synchronized SimulationSubsystem getInstance() {
  //       if (simSub == null) {
  //           simSub = new SimulationSubsystem();
  //       }

  //       return simSub;
  //   }


  /** Creates a new SimulationSubsystem. */
  public SimulationSubsystem() {

    populateNotes();
    

   currentPose = new Pose2d();

   collectionTimer = new Timer();

   shootTimer = new Timer();

   stateHandler.setBBTwoCovered(true);

   stateHandler.setBBThreeCovered(true);

  }

  public void populateNotes(){
    notePoses = new ArrayList<>();

    notePoses.add(new Translation2d(2.89, 4.10)); //podiumLoc
    notePoses.add(new Translation2d(2.89, 5.56)); //midLoc
    notePoses.add(new Translation2d(2.89, 7.00)); //ampLoc

    notePoses.add(new Translation2d(8.28, 7.45)); //1Loc
    notePoses.add(new Translation2d(8.28, 5.78)); //2Loc
    notePoses.add(new Translation2d(8.28, 4.09)); //3Loc
    notePoses.add(new Translation2d(8.28, 2.44)); //4Loc
    notePoses.add(new Translation2d(8.28,0.76)); //5Loc

  }

  public void updatePose(Pose2d robotPose){
    currentPose = robotPose;
  }

  public void setShooting(boolean shooting){
    isShooting = shooting;
  }

  public void setCollecting(boolean collecting){
    isCollecting = collecting;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (Utils.isSimulation()){
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

        collectionTimer.stop();
        collectionTimer.reset();
      }

    }
    else if (isShooting){

      if (shootTimer.get() == 0){
        shootTimer.start();
        stateHandler.setBBFourCovered(true);
      }

      if (shootTimer.get() > ShootTimes.bb2OffTime){
        stateHandler.setBBTwoCovered(false);
      }

      if (shootTimer.get() > ShootTimes.bb3OffTime){
        stateHandler.setBBThreeCovered(false);
      }

      if (shootTimer.get() > ShootTimes.bb4OffTime){
        stateHandler.setBBFourCovered(false);
      }

    }
    else{
      for (int i = notePoses.size()-1; i >=0; i--){
        Translation2d notePos = notePoses.get(i);
        if (stateHandler.getDesiredIntakeState() == IntakeStates.DEPLOYED && stateHandler.getCurrentIntakeRollerSpeed() == IntakeRollerSpeeds.INTAKE
            && notePos.getDistance(currentPose.getTranslation())< collectionDist){

          isCollecting = true;

          notePoses.remove(i);
        }
      }
    }

    SmartDashboard.putNumber("collecting timer", collectionTimer.get());

    SmartDashboard.putBoolean("isCollecting", isCollecting);

    SmartDashboard.putString("sim pose", currentPose.toString());
    }

    SmartDashboard.putString("SIM SUBSYSTEM RUNNING", "!!!!!!");

    

  }
}
