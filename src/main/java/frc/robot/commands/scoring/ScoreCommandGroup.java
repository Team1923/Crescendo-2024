package frc.robot.commands.Scoring;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Swerve.GoalCentricCommand;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ScoreCommandGroup extends ParallelCommandGroup {
  /** Creates a new ScoreCommandGroup. */
  private StateHandler stateHandler = StateHandler.getInstance();

  public ScoreCommandGroup(CommandSwerveDrivetrain swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    addCommands(
        new GoalCentricCommand(swerve, translationSup, strafeSup, rotationSup),
        new ScoreGamePiece());
  }

}