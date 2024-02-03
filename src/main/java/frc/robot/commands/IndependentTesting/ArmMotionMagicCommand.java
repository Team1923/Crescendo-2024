// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndependentTesting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.RobotStateUtils.StateVariables.ArmStates;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMotionMagicCommand extends Command {

  ArmSubsystem armSubsystem;

  /** Creates a new ArmMotionMagicCommand. */
  public ArmMotionMagicCommand(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.

    armSubsystem = arm;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setArmPosition(ArmStates.SPEAKER.getArmPosition().getAngularSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArmPosition(ArmStates.STOWED.getArmPosition().getAngularSetpoint());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
