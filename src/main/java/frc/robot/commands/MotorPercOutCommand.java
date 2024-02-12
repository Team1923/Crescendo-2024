// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;

public class MotorPercOutCommand extends Command {
  /** Creates a new MotorPercOutCommand. */

  private MotorSubsystem motor;
  private DoubleSupplier percOut;
  
  public MotorPercOutCommand(MotorSubsystem motor, DoubleSupplier percOut) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.motor = motor;
    this.percOut = percOut;

    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor.setPercOut((Math.abs(percOut.getAsDouble())>0.05) ? percOut.getAsDouble() : 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
