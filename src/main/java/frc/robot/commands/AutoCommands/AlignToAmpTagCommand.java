// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.VisionUtil.LimelightInterface;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToAmpTagCommand extends Command {

  LimelightInterface limelight = LimelightInterface.getInstance();
  SwerveSubsystem swerve;
  DoubleSupplier horizontalTranslationSupplier;
  PIDController rotationController = new PIDController(0.02, 0, 0);
  PIDController strafeController = new PIDController(0.01, 0, 0);

  /** Creates a new AllignToTagCommand. */
  public AlignToAmpTagCommand(SwerveSubsystem s, DoubleSupplier horizTranslation) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = s;
    horizontalTranslationSupplier = horizTranslation;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationValue = MathUtil.applyDeadband(horizontalTranslationSupplier.getAsDouble(), Swerve.stickDeadband);
    double rotationGoal = DriverStation.getAlliance().get() == Alliance.Blue ? -90 : 90;
              //CHECK NEGATIVE
    swerve.drive(new Translation2d(-strafeController.calculate(limelight.getXAngleOffset(), 0), translationValue).times(Swerve.maxSpeed), 
                                  rotationController.calculate(swerve.getGyroYaw().getDegrees(), rotationGoal) * Swerve.maxAngularVelocity, true, true);

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
