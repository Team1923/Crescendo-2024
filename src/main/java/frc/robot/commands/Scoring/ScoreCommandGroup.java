// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.RobotStateUtils.StateHandler;
import frc.robot.commands.Swerve.GoalCentricCommand;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCommandGroup extends ParallelCommandGroup {
  /** Creates a new ScoreCommandGroup. */
  private StateHandler stateHandler = StateHandler.getInstance();

  public ScoreCommandGroup(SwerveSubsystem swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    /* No alignment needed conditions. */
    if (stateHandler.getScoreInAmp() || stateHandler.getWantToPositionForSubwoofer()) {
      addCommands(
          new ScoreGamePiece());
    } 
    /* Alignment is needed. */
    else {
      addCommands(
          new GoalCentricCommand(swerve, translationSup, strafeSup, rotationSup),
          new ScoreGamePiece()
      );
    }

  }

}
