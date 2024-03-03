package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.FeederSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeRollerSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeStates;

public class DeployIntakeCommand extends Command {
  
  StateHandler stateHandler = StateHandler.getInstance();
  /** Creates a new DeployIntakeCommand. */
  public DeployIntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setDesiredIntakeState(IntakeStates.DEPLOYED);
    stateHandler.setDesiredIntakeRollerSpeed(IntakeRollerSpeeds.INTAKE);
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.INWARD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredIntakeState(IntakeStates.STOWED);
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.OFF);
    stateHandler.setDesiredIntakeRollerSpeed(IntakeRollerSpeeds.OFF);
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.getBBThreeCovered();
  }


  
}
