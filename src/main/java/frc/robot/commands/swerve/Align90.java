// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.Limelight.LimelightInterface;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class Align90 extends Command {
  
  /* Swerve + Limelight Initialization */
  private CommandSwerveDrivetrain swerve;
  private SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private LimelightInterface limelight = LimelightInterface.getInstance();
  private StateHandler stateHandler = StateHandler.getInstance();

  /* Fancy double stuff for input + output to swerve. */
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;

  double tolerance = 3;

  /* PID Things */
 
  private PIDController rotationController;

  /** Creates a new GoalCentricCommand. */
  public Align90(CommandSwerveDrivetrain swerve, DoubleSupplier t, DoubleSupplier s) {
    this.swerve = swerve;
    this.translationSup = t;
    this.strafeSup = s;

    rotationController = new PIDController(SwerveConstants.headingKP, SwerveConstants.headingKI, SwerveConstants.headingKD);

    rotationController.enableContinuousInput(-180, 180);
    addRequirements(this.swerve);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationValue = Math.abs(translationSup.getAsDouble()) > 0.1 ? translationSup.getAsDouble() : 0;
    double strafeValue = Math.abs(strafeSup.getAsDouble()) > 0.1 ? strafeSup.getAsDouble() : 0;
    double rotValue = rotationController.calculate(swerve.getGyroYaw().getDegrees(), (DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : -90);

    // SmartDashboard.putNumber("ROT VAL", rotValue);

      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translationValue * SwerveConstants.maxSpeed, 
      strafeValue  * SwerveConstants.maxSpeed, 
      rotValue * SwerveConstants.maxAngularRate, swerve.getGyroYaw()); 
    
      swerve.setControl(drive.withSpeeds(chassisSpeeds));
    
    

    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(((DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : -90)-swerve.getGyroYaw().getDegrees()) < tolerance);
  }
}
