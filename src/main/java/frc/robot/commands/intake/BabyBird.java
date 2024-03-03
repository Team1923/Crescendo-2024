// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;
import frc.robot.lib.StateMachine.StateVariables.FeederSpeeds;
import frc.robot.lib.StateMachine.StateVariables.ShooterSpeeds;

public class BabyBird extends Command {

  StateHandler stateHandler = StateHandler.getInstance();

  boolean hasPiece;

  /** Creates a new BabyBird. */
  public BabyBird() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setDesiredArmState(ArmStates.BABY_BIRD);
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.OUTWARD);
    stateHandler.setDesiredShootingSpeed(ShooterSpeeds.BABY_BIRD);
    hasPiece = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stateHandler.getBBFourCovered()){
      hasPiece = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredArmState(ArmStates.STOWED);
    stateHandler.setDesiredFeederSpeed(FeederSpeeds.OFF);
    stateHandler.setDesiredShootingSpeed(ShooterSpeeds.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasPiece && !stateHandler.getBBFourCovered();
  }
}
