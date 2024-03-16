// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.desired_scoring_location;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.StateMachine.StateHandler;

public class SetArmToTrap extends Command {
  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new SetArmToRanged. */
  public SetArmToTrap() {
    // Use addRequirements() here to declare subsystem dependencies.


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    stateHandler.setScoreInTrap(true);
    stateHandler.setScoreInAmp(false);
    stateHandler.setScoreInSubwoofer(false);
    stateHandler.setScoreInReverseSubwoofer(false);
    stateHandler.setWantPunt(false);
    stateHandler.setWantUnguardable(false);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.getScoreInTrap() ==false;
  }
}
