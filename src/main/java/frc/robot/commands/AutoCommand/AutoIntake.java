package frc.robot.commands.AutoCommand;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.SimUtils.SimulationSubsystem;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.FeederSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeRollerSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeStates;

public class AutoIntake extends Command {
  
  StateHandler stateHandler = StateHandler.getInstance();

  boolean hasPickedGP; // may not need it based on how we changed how paths are made(path + intake in parallel, ask griffin)

  /** Creates a new DeployIntakeCommand. */
  public AutoIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setDesiredIntakeState(IntakeStates.DEPLOYED);
    stateHandler.setDesiredIntakeRollerSpeed(IntakeRollerSpeeds.INTAKE);
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.INWARD);
    hasPickedGP =false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stateHandler.getBBOneCovered()){
      hasPickedGP = true;
   }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredIntakeState(IntakeStates.STOWED);
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.OFF);
    stateHandler.setDesiredIntakeRollerSpeed(IntakeRollerSpeeds.OFF);

    if (Utils.isSimulation()){
      SimulationSubsystem.getInstance().setCollecting(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasPickedGP && stateHandler.getBBThreeCovered() && !stateHandler.getBBFourCovered();
  }
}
