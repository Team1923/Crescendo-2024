// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndependentTesting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeRollerCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  /** Creates a new IntakeRollerCommand. */
  public RunIntakeRollerCommand(IntakeSubsystem i) {
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
    intakeSubsystem.setRollerWheelSpeed(0.2, 0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setRollerWheelSpeed(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}