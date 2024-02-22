// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  /* Controller Instantiations */
  private final CommandXboxController driverXboxController = new CommandXboxController(0);
  private final CommandPS4Controller operatorPS4Controller = new CommandPS4Controller(1);

  /* Subsystem Instantiations */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.maxSpeed * 0.1).withRotationalDeadband(SwerveConstants.maxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(SwerveConstants.maxSpeed);

  private void configureBindings() {
    /* Default Swerve Drive Command */
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverXboxController.getLeftY() * SwerveConstants.maxSpeed)
            .withVelocityY(-driverXboxController.getRightY() * SwerveConstants.maxSpeed)
            .withRotationalRate(-driverXboxController.getRightX() * SwerveConstants.maxAngularRate)));

    /* Zero the Gyro when pressing Y on the XBOX Controller */
    driverXboxController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    /* Simulation tool for Swerve */
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  /* TODO: update with our custom pathplanner implementation! */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
