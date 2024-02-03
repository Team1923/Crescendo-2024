
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndependentTesting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.RobotStateUtils.StateHandler;

public class ActivateBeamBreak extends Command {

  StateHandler stateHandler = StateHandler.getInstance();

  private int BBID;

  /** Creates a new ActivateBeamBreak. */
  public ActivateBeamBreak(int BBID) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.BBID = BBID;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (BBID == 1){
      stateHandler.setBBOneCovered(true);
    }
    if (BBID == 2){
      stateHandler.setBBTwoCovered(true);
    }
    if (BBID == 3){
      stateHandler.setBBThreeCovered(true);
    }
    if (BBID == 4){
      stateHandler.setBBFourCovered(true);
    }
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (BBID == 1){
      stateHandler.setBBOneCovered(false);
    }
    if (BBID == 2){
      stateHandler.setBBTwoCovered(false);
    }
    if (BBID == 3){
      stateHandler.setBBThreeCovered(false);
    }
    if (BBID == 4){
      stateHandler.setBBFourCovered(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
