// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;


import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.RobotStateUtils.StateHandler;
import frc.lib.RobotStateUtils.StateVariables.ArmStates;

public class PositionToAmp extends Command {
  StateHandler stateHandler = StateHandler.getInstance();
  /** Creates a new AmpCommand. */
  public PositionToAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stateHandler.setDesiredArmState(ArmStates.AMP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredArmState(ArmStates.STOWED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !(stateHandler.getBBTwoCovered() && stateHandler.getBBThreeCovered());
  }
}
