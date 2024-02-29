package frc.robot.commands.scoring;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.swerve.GoalCentricCommand;
import frc.robot.commands.swerve.TrapCentricCommand;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ScoreCommandGroup extends ParallelCommandGroup {
  /** Creates a new ScoreCommandGroup. */
  
   StateHandler stateHandler = StateHandler.getInstance();

  public ScoreCommandGroup(CommandSwerveDrivetrain swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
        if (stateHandler.getScoreInAmp()){
          addCommands(new ScoreGamePiece());
        }
        else{
          addCommands(new GoalCentricCommand(swerve, translationSup, strafeSup, rotationSup),
          new ScoreGamePiece());
        }
  }

}
