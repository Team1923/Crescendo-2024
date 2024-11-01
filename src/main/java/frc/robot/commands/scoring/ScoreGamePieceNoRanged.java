package frc.robot.commands.scoring;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;
import frc.robot.lib.StateMachine.StateVariables.FeederSpeeds;
import frc.robot.lib.StateMachine.StateVariables.ShooterSpeeds;

public class ScoreGamePieceNoRanged extends Command {
  
  StateHandler stateHandler = StateHandler.getInstance();
  private Timer frontAmpTimer = new Timer();
  private Timer inputTimer = new Timer();
  /** Creates a new SpeakerPositionCommand. */
  public ScoreGamePieceNoRanged() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (stateHandler.getScoreInAmp()) {
      stateHandler.setDesiredArmState(ArmStates.AMP);
    } else if (stateHandler.getScoreInTrap()){
      stateHandler.setDesiredArmState(ArmStates.TRAP);
      stateHandler.setDesiredShootingSpeed(ShooterSpeeds.TRAP);
    } else if(stateHandler.getWantUnguardable()){
      stateHandler.setDesiredArmState(ArmStates.UNGUARDABLE);
      stateHandler.setDesiredShootingSpeed(ShooterSpeeds.UNGUARDABLE_SHOT);
      stateHandler.setBlowPower(1);
    } else if(stateHandler.getWantFrontAmp()){
      stateHandler.setDesiredArmState(ArmStates.FRONT_AMP);
      stateHandler.setDesiredShootingSpeed(ShooterSpeeds.FRONT_AMP_SHOT);
      stateHandler.setBlowPower(1);
    }
    else if (stateHandler.getScoreInSubwoofer() || stateHandler.getScoreInReverseSubwoofer()){
      stateHandler.setDesiredArmState(ArmStates.SPEAKER);
      stateHandler.setDesiredShootingSpeed(ShooterSpeeds.SHOOT);
    }
    inputTimer.start();
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stateHandler.getCurrentArmState() == ArmStates.AMP) {
      stateHandler.setDesiredFeederSpeed(FeederSpeeds.OUTWARD);
    }
    if(!stateHandler.getBBThreeCovered()){
      frontAmpTimer.start();
    }

    if(inputTimer.get() > 1){
      stateHandler.setOperatorInputTimingGood(true);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredArmState(ArmStates.STOWED);
    stateHandler.setDesiredShootingSpeed(ShooterSpeeds.IDLE);
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.OFF);
    stateHandler.setBlowPower(0);
    frontAmpTimer.stop();
    frontAmpTimer.reset();
    inputTimer.stop();
    inputTimer.reset();
    stateHandler.setOperatorInputTimingGood(false);
  }
  


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //when you let go of trigger, automatically ends
    // if(!stateHandler.getIsGoalCentric()){
    //   return true;
    // }
    if (stateHandler.getScoreInAmp()) {
      return !stateHandler.getBBTwoCovered() && !stateHandler.getBBThreeCovered();
    } else if(stateHandler.getWantFrontAmp()){
      return frontAmpTimer.get() > 5 && !stateHandler.getBBTwoCovered() && !stateHandler.getBBThreeCovered() && !stateHandler.getBBFourCovered();
    }
    else {
      return !stateHandler.getBBTwoCovered() && !stateHandler.getBBThreeCovered() && !stateHandler.getBBFourCovered();
    }
    
  }
}
