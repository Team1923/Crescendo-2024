package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;
import frc.robot.lib.StateMachine.StateVariables.FeederSpeeds;
import frc.robot.lib.StateMachine.StateVariables.ShooterSpeeds;

public class ScoreGamePiece extends Command {
  StateHandler stateHandler = StateHandler.getInstance();
  /** Creates a new SpeakerPositionCommand. */
  public ScoreGamePiece() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (stateHandler.getScoreInAmp()) {
      stateHandler.setDesiredArmState(ArmStates.AMP);
    } else {
      stateHandler.setDesiredArmState(ArmStates.SPEAKER);
      stateHandler.setDesiredShootingSpeed(ShooterSpeeds.SHOOT);
    }
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stateHandler.getCurrentArmState() == ArmStates.AMP) {
      stateHandler.setDesiredFeederSpeed(FeederSpeeds.OUTWARD);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredArmState(ArmStates.STOWED);
    stateHandler.setDesiredShootingSpeed(ShooterSpeeds.IDLE);
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //when you let go of trigger, automatically ends
    if (stateHandler.getScoreInAmp()) {
      return !stateHandler.getBBTwoCovered() && !stateHandler.getBBThreeCovered();
    } else {
      return !stateHandler.getBBTwoCovered() && !stateHandler.getBBThreeCovered() && !stateHandler.getBBFourCovered();
    }
    
  }
}
