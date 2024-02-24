package frc.robot.commands.Desired_Scoring_Location;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.StateMachine.StateHandler;

public class SetArmToAmp extends Command {
  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new SetArmToAmp. */
  public SetArmToAmp() {
    // Use addRequirements() here to declare subsystem dependencies.


    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setScoreInAmp(true);
    stateHandler.setWantToPositionForSubwoofer(false);
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
    
    return stateHandler.getScoreInAmp() == false;
  }
}
