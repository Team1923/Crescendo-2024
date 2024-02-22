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
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  /* Constants needed for Swerve - DO NOT CHANGE! */
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = (1.5 * Math.PI) * 8;

  /* Controller Instantiations */
  private final CommandXboxController driverXboxController = new CommandXboxController(0);
  private final CommandPS4Controller operatorPS4Controller = new CommandPS4Controller(1);

  /* Subsystem Instantiations */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    /* Default Swerve Drive Command */
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(getSwerveJoystickInput()[0] * MaxSpeed)
            .withVelocityY(getSwerveJoystickInput()[1] * MaxSpeed)
            .withRotationalRate(getSwerveJoystickInput()[2] * MaxAngularRate)));

    /* Zero the Gyro when pressing Y on the XBOX Controller */
    driverXboxController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    /* Simulation tool for Swerve */
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /* Helper method that does some inversion based on the alliance color. */
  public double[] getSwerveJoystickInput() {
    /* 
     * Index 0: driverXboxController.getLeftY();
     * Index 1: driverXboxController.getLeftX();
     * Index 2: driverXboxController.getRightX();
     */
    double[] blueJoystickValues = {
        -driverXboxController.getLeftY(), 
        -driverXboxController.getLeftX(), 
        -driverXboxController.getRightX()};

    double[] redJoystickValues = {
      driverXboxController.getLeftY(), 
      driverXboxController.getLeftX(), 
      -driverXboxController.getRightX()};
    
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return redJoystickValues;
    } else {
      return blueJoystickValues;
    }
  }

  public RobotContainer() {
    configureBindings();
  }

  /* TODO: update with our custom pathplanner implementation! */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
