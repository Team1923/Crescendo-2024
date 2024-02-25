// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualHashTuningCommands;


import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.StateMachine.StateHandler;

public class ChangeRPMOffset extends Command {

  double offset;
  Timer delay;
  /** Creates a new ChangePositionOffset. */
  public ChangeRPMOffset(double offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.offset = offset;
    delay = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delay.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (delay.get() > 0.2){
      StateHandler.getInstance().setRPMOffset(StateHandler.getInstance().getRPMOffset() + offset);
      delay.reset();
      delay.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
