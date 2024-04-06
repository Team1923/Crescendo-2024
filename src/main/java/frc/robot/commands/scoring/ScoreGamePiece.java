package frc.robot.commands.scoring;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;
import frc.robot.lib.StateMachine.StateVariables.FeederSpeeds;
import frc.robot.lib.StateMachine.StateVariables.ShooterSpeeds;

public class ScoreGamePiece extends Command {
  
  StateHandler stateHandler = StateHandler.getInstance();
  private Timer inputTimer = new Timer();
  private Timer trapTimer = new Timer();
  private Timer trapTimer2 = new Timer();
 
  /** Creates a new SpeakerPositionCommand. */
  public ScoreGamePiece() {
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
      stateHandler.setBlowPower(1);
    }
      else if(stateHandler.getWantUnguardable()){
        stateHandler.setDesiredArmState(ArmStates.UNGUARDABLE);
        stateHandler.setDesiredShootingSpeed(ShooterSpeeds.UNGUARDABLE_SHOT);
      }
    else {
      stateHandler.setDesiredArmState(ArmStates.SPEAKER);
      stateHandler.setDesiredShootingSpeed(ShooterSpeeds.SHOOT);
    }

    if (stateHandler.getScoreInTrap()){
      inputTimer.start();

    }
    
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(stateHandler.getIsCenteredToTag() && !stateHandler.getScoreInTrap()){
      inputTimer.start();
    }
  

    if (inputTimer.get() > Constants.ArmConstants.armSettleTime && !stateHandler.getScoreInTrap() ) {
      stateHandler.setOperatorInputTimingGood(true);
    }


    if(stateHandler.getCurrentArmState() == ArmStates.TRAP){
      trapTimer2.start();
    }

    if(stateHandler.getScoreInTrap()){
      if(inputTimer.get() > Constants.ArmConstants.armSettleTime && trapTimer2.get() > 0.5){
        stateHandler.setOperatorInputTimingGood(true);
      }
    }

    if (stateHandler.getCurrentArmState() == ArmStates.AMP) {
      stateHandler.setDesiredFeederSpeed(FeederSpeeds.OUTWARD);
    }

    if(!stateHandler.getBBThreeCovered()){
      trapTimer.start();
    }

    
    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredArmState(ArmStates.STOWED);
    stateHandler.setDesiredShootingSpeed(ShooterSpeeds.IDLE);
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.OFF);
    stateHandler.setBlowPower(0);
    inputTimer.stop();
    inputTimer.reset();
    trapTimer.stop();
    trapTimer.reset();
    trapTimer2.stop();
    trapTimer2.reset();
    stateHandler.setOperatorInputTimingGood(false);
  }
  


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //when you let go of trigger, automatically ends
    if (stateHandler.getScoreInAmp()) {
      return !stateHandler.getBBTwoCovered() && !stateHandler.getBBThreeCovered();
    } else if(stateHandler.getScoreInTrap()){
      return  trapTimer.get() > 7 && !stateHandler.getBBTwoCovered() && !stateHandler.getBBThreeCovered() && !stateHandler.getBBFourCovered();
    }
    else {
      return !stateHandler.getBBTwoCovered() && !stateHandler.getBBThreeCovered() && !stateHandler.getBBFourCovered();
    }
    
  }
}
