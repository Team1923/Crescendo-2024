// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;
import frc.robot.lib.StateMachine.StateVariables.IntakeStates;

public class ArmUpToClimb extends Command {

  StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new ArmUpToClimb. */
  public ArmUpToClimb() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setDesiredArmState(ArmStates.CLIMB);

    // stateHandler.setDesiredIntakeState(IntakeStates.DEPLOYED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("ArmUpToClimb", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("ArmUpToClimb", false);
    if (stateHandler.getCurrentArmState() == ArmStates.CLIMB){
          stateHandler.setManuallyClimbing(true);
    }
    else{
      stateHandler.setManuallyClimbing(false);
      stateHandler.setDesiredArmState(ArmStates.STOWED);
      // stateHandler.setDesiredIntakeState(IntakeStates.STOWED);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.getCurrentArmState() == ArmStates.CLIMB;
  }
}
