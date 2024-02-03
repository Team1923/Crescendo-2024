// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndependentTesting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmPercOutCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  DoubleSupplier intakeControl;
  /** Creates a new IntakeArmPercOutCommand. */
  public IntakeArmPercOutCommand(IntakeSubsystem i, DoubleSupplier ds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = i; 
    intakeControl = ds;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setIntakeArmPercentOut(intakeControl.getAsDouble());
    SmartDashboard.putNumber("Joystick/Max gravity for Intake Arm", intakeControl.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeArmPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
