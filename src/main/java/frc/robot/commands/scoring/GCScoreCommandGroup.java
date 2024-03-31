package frc.robot.commands.scoring;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.swerve.GoalCentricCommand;
import frc.robot.commands.swerve.TrapCentricCommand;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GCScoreCommandGroup extends ParallelCommandGroup {
  /** Creates a new ScoreCommandGroup. */
  
   StateHandler stateHandler = StateHandler.getInstance();
  

  public GCScoreCommandGroup(CommandSwerveDrivetrain swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
        if (stateHandler.getScoreInTrap()){
          addCommands(new TrapCentricCommand(swerve, translationSup, strafeSup, rotationSup),
          new ScoreGamePiece());
        }
        else{
          addCommands(new GoalCentricCommand(swerve, translationSup, strafeSup, rotationSup),
          new ScoreGamePiece());
        }
          
        
  }
  




}
