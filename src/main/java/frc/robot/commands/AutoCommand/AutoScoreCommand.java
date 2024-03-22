// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.SimUtils.SimulationSubsystem;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;
import frc.robot.lib.StateMachine.StateVariables.FeederSpeeds;
import frc.robot.lib.StateMachine.StateVariables.ShooterSpeeds;

public class AutoScoreCommand extends Command {
  
  StateHandler stateHandler = StateHandler.getInstance();
  private Timer inputTimer = new Timer();
  private Timer shotTimer = new Timer();
  
  /** Creates a new SpeakerPositionCommand. */
  public AutoScoreCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   

    if (stateHandler.getScoreInAmp()) {
      stateHandler.setDesiredArmState(ArmStates.AMP);
    } else {
      stateHandler.setDesiredArmState(ArmStates.SPEAKER);
    }
    inputTimer.start();
    shotTimer.start();
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stateHandler.getCurrentArmState() == ArmStates.AMP) {
      stateHandler.setDesiredFeederSpeed(FeederSpeeds.OUTWARD);
    }

    if (!stateHandler.getBBFourCovered()){
      stateHandler.setDesiredShootingSpeed(ShooterSpeeds.SHOOT);

    }

    
    if (inputTimer.get() > Constants.ArmConstants.armSettleTime) {
      stateHandler.setOperatorInputTimingGood(true);
    }

    if(shotTimer.get() > 1.5){
      stateHandler.setAutoOverride(true);
    }


   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredArmState(ArmStates.STOWED);
    stateHandler.setDesiredShootingSpeed(ShooterSpeeds.IDLE);
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.OFF);
    inputTimer.stop();
    inputTimer.reset();
    shotTimer.stop();
    shotTimer.reset();
    stateHandler.setOperatorInputTimingGood(false);
    stateHandler.setAutoOverride(false);
    stateHandler.setIsCenteredToTag(false);

  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return !stateHandler.getBBTwoCovered() && !stateHandler.getBBThreeCovered() && !stateHandler.getBBFourCovered();  
  }
}
