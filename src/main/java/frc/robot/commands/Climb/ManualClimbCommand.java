// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.ArmSubsystem;

public class ManualClimbCommand extends Command {

  ArmSubsystem armSubsystem;
  DoubleSupplier input;
  StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new ManualClimbCommand. */
  public ManualClimbCommand(ArmSubsystem arm, DoubleSupplier input) {
    // Use addRequirements() here to declare subsystem dependencies.

    armSubsystem = arm;
    this.input = input;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armPosition = armSubsystem.getArmPositionRads();
    if (stateHandler.getManuallyClimbing() && Math.abs(input.getAsDouble()) > 0.1){
      armSubsystem.setPercentOut(input.getAsDouble());  
    }
    else{
      armSubsystem.setArmPosition(armPosition);
    }
    SmartDashboard.putNumber("Joystick Input", input.getAsDouble());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setManuallyClimbing(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
