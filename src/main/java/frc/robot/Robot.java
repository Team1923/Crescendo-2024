// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.desired_scoring_location.SetArmToAmp;
import frc.robot.commands.desired_scoring_location.SetArmToRanged;
import frc.robot.commands.desired_scoring_location.SetArmToSubwoofer;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.scoring.ScoreCommandGroup;
import frc.robot.commands.scoring.ScoreGamePiece;
import frc.robot.lib.Autonomous.AutoChooser;
import frc.robot.lib.Limelight.LimelightInterface;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /* Chooser Initialization */
  public AutoChooser selector;

  @Override
  public void robotInit() {
    NamedCommands.registerCommand("DeployIntake", new DeployIntakeCommand());
    NamedCommands.registerCommand("ScoreGamePiece", new ScoreGamePiece());
    NamedCommands.registerCommand("wantSubwoofer", new SetArmToSubwoofer());
    NamedCommands.registerCommand("wantRange", new SetArmToRanged());


    m_robotContainer = new RobotContainer();
    this.selector = new AutoChooser();   
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.initializeAuto(selector);
    LimelightInterface.getInstance().aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("YAW2", m_robotContainer.drivetrain.getGyroYaw().getDegrees());
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
