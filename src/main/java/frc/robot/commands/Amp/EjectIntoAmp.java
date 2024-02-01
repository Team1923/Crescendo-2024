// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.RobotStateUtils.StateHandler;
import frc.lib.RobotStateUtils.StateVariables.FeederSpeeds;

public class EjectIntoAmp extends Command {
  /** Creates a new AmpScore. */
  private StateHandler stateHandler = StateHandler.getInstance();
  public EjectIntoAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.OUTWARD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !stateHandler.getBBTwoCovered() && !stateHandler.getBBThreeCovered();
  }
}
