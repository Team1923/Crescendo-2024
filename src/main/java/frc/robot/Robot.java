// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.AutoCommand.AutoIntake;
import frc.robot.commands.AutoCommand.AutoScoreCommand;
import frc.robot.commands.AutoCommand.AutoScoreCommandGroup;
import frc.robot.commands.AutoCommand.AutoSetArmToRanged;
import frc.robot.commands.AutoCommand.AutoSetArmToSubwoofer;
import frc.robot.commands.desired_scoring_location.SetArmToRanged;
import frc.robot.commands.desired_scoring_location.SetArmToSubwoofer;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.scoring.GCScoreCommandGroup;
import frc.robot.commands.scoring.ScoreGamePiece;
import frc.robot.lib.Autonomous.AutoChooser;
import frc.robot.lib.Limelight.LimelightInterface;
import frc.robot.lib.SimUtils.SimulationSubsystem;
import frc.robot.lib.StateMachine.StateHandler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  StateHandler stateHandler = StateHandler.getInstance();

  SimulationSubsystem sim;

  /* Chooser Initialization */
  public AutoChooser selector;

  private DigitalInput outsideButton = new DigitalInput(ButtonConstants.outsideButton);
  private DigitalInput insideButton = new DigitalInput(ButtonConstants.insideButton);





  @Override
  public void robotInit() {
    NamedCommands.registerCommand("DeployIntake", new AutoIntake());
    NamedCommands.registerCommand("ScoreGamePiece", new AutoScoreCommand());
    NamedCommands.registerCommand("wantSubwoofer", new AutoSetArmToSubwoofer());
    NamedCommands.registerCommand("wantRange", new AutoSetArmToRanged());




    NamedCommands.registerCommand("autoOverride", new InstantCommand(() -> stateHandler.setAutoOverride(true)));


    CameraServer.startAutomaticCapture();



    m_robotContainer = new RobotContainer();
    this.selector = new AutoChooser();   


    if (Utils.isSimulation()){
      sim = sim.getInstance();
    }

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (Utils.isSimulation()){
          sim.updatePose(m_robotContainer.drivetrain.getState().Pose);
    }
    
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // if (outsideButton.get()){
    //   m_robotContainer.armSubsystem.setArmCoast();
    //   m_robotContainer.intakeSubsystem.setIntakeArmCoast();

    // }
    // else{
    //   m_robotContainer.armSubsystem.setArmBrake();
    //   m_robotContainer.intakeSubsystem.setIntakeArmBrake();

    // }
    // if (insideButton.get()){
    //   m_robotContainer.drivetrain.zeroGyro();
    //   m_robotContainer.armSubsystem.zeroArm();
    //   m_robotContainer.intakeSubsystem.zeroIntakeArm();
    // }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.initializeAuto(selector);
    LimelightInterface.getInstance().aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    
    if (stateHandler.getBBOneCovered()){
      stateHandler.setBB1Dead(true);
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    if(Utils.isSimulation()){
      SimulationSubsystem.getInstance().populateNotes();
      stateHandler.setBBThreeCovered(true);
      stateHandler.setBBTwoCovered(true);
    }


  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    stateHandler.setAutoOverride(false);
   
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
        StateHandler.getInstance().setAutoOverride(false);

  }

  @Override
  public void teleopPeriodic() {


    //swerve doesn't have a periodic in Tuner X
    m_robotContainer.drivetrain.checkCurrentLimits();

    /*
     * HASHMAP TUNING
     */

     
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
