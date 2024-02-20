// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.RobotStateUtils.StateHandler;

public class SetSubWoofer extends Command {
  /** Creates a new RequireSubwooferPosition. */
  private StateHandler stateHandler = StateHandler.getInstance();
  public SetSubWoofer() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* This is an operator button. Forces the arm to always score in the subwoofer. */
    stateHandler.setWantToPositionForSubwoofer(true);
    stateHandler.setScoreInAmp(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.getWantToPositionForSubwoofer() == false;
  }
}
