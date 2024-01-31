// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.RobotStateUtils.StateHandler;
import frc.lib.RobotStateUtils.StateVariables.IntakeRollerSpeeds;
import frc.lib.RobotStateUtils.StateVariables.IntakeStates;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredIntakeState(IntakeStates.STOWED);
    stateHandler.setDesiredIntakeRollerSpeed(IntakeRollerSpeeds.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.getBBThreeCovered();
  }
}
