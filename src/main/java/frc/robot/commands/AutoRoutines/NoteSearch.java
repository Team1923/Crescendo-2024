// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.swerve.ManageRequests;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.SwerveRequests;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class NoteSearch extends Command {

  StateHandler stateHandler = StateHandler.getInstance();

  public enum Direction{
    RIGHT(1),
    LEFT(-1);

    private int val;
    private Direction(int val){
      this.val = val;
    }
  }

  CommandSwerveDrivetrain swerve;


  /** Creates a new NoteSearch. */
  public NoteSearch(CommandSwerveDrivetrain swerve, Direction nextNoteDirection) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setCurrentSwerveRequest(SwerveRequests.NOTE_SEARCHING);

    swerve.setControl(((SwerveRequest.FieldCentricFacingAngle)SwerveRequests.NOTE_SEARCHING.request)
            .withVelocityX(0)
            .withVelocityY(0.5 * ManageRequests.sideInversions()[0] * SwerveConstants.maxSpeed)
            .withTargetDirection(Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) ? 45 : 45)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setCurrentSwerveRequest(SwerveRequests.AUTO);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.getBBOneCovered() || stateHandler.getBBTwoCovered() || stateHandler.getBBThreeCovered();
  }
}
