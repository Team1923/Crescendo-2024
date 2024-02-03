// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndependentTesting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.RobotStateUtils.StateHandler;
import frc.lib.RobotStateUtils.StateVariables.IntakePosition;
import frc.lib.RobotStateUtils.StateVariables.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMotionMagicCommand extends Command {
  
  IntakeSubsystem intakeSubsystem;
  StateHandler stateHandler = StateHandler.getInstance();
  /** Creates a new IntakeMotionMagicCommand. */
  public IntakeMotionMagicCommand(IntakeSubsystem i) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = i;
    addRequirements(i);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setIntakePosition(IntakeStates.DEPLOYED.getIntakePosition().getAngularSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakePosition(IntakeStates.STOWED.getIntakePosition().getAngularSetpoint());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
