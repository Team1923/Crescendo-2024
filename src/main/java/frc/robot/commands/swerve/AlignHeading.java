// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.sql.Driver;
import java.util.concurrent.locks.Lock;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.Limelight.LimelightInterface;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;


/* TODO: cross check with existing code to get state handler integration going */
public class AlignHeading extends Command {
  
  /* Swerve + Limelight Initialization */
  private CommandSwerveDrivetrain swerve;
  private SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private StateHandler stateHandler = StateHandler.getInstance();

  /* Fancy double stuff for input + output to swerve. */
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;

  double tolerance = 2;

  /* PID Things */
 
  private PIDController rotationController;

  double desiredHeading;



  /** Creates a new GoalCentricCommand. */
  public AlignHeading(CommandSwerveDrivetrain swerve, DoubleSupplier t, DoubleSupplier s) {
    this.swerve = swerve;
    this.translationSup = t;
    this.strafeSup = s;

    this.desiredHeading = LimelightSubsystem.getSeenTrapHeading();

    rotationController = new PIDController(SwerveConstants.headingKP, SwerveConstants.headingKI, SwerveConstants.headingKD);

    rotationController.enableContinuousInput(-180, 180);
    addRequirements(this.swerve);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(stateHandler.getManuallyClimbing()){
      desiredHeading = LimelightSubsystem.roundToClosestClimbHeading(stateHandler.getCurrentRobotHeading());
    }
    else if (stateHandler.getWantFrontAmp() || stateHandler.getScoreInAmp()) {
        desiredHeading = DriverStation.getAlliance().get() == Alliance.Blue ? -90 : 90;
    } else if (stateHandler.getScoreInTrap()) {
        desiredHeading = LimelightSubsystem.roundToClosestTrapHeading(stateHandler.getCurrentRobotHeading());
        //desiredHeading = LimelightSubsystem.roundToClosestHeading2(stateHandler.getCurrentRobotHeading());
    } 
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationValue = Math.abs(translationSup.getAsDouble()) > 0.1 ? translationSup.getAsDouble() : 0;
    double strafeValue = Math.abs(strafeSup.getAsDouble()) > 0.1 ? strafeSup.getAsDouble() : 0;
    double rotValue = (desiredHeading == -1) ?  0 : rotationController.calculate(stateHandler.getCurrentRobotHeading(), desiredHeading);

    ChassisSpeeds chassisSpeeds = DriverStation.getAlliance().get() == Alliance.Blue ? ChassisSpeeds.fromFieldRelativeSpeeds(translationValue * SwerveConstants.maxSpeed, 
    strafeValue  * SwerveConstants.maxSpeed, 
    rotValue * SwerveConstants.maxAngularRate, swerve.getGyroYaw()) : ChassisSpeeds.fromFieldRelativeSpeeds(-1 * translationValue * SwerveConstants.maxSpeed, 
     -1* strafeValue  * SwerveConstants.maxSpeed, 
    rotValue * SwerveConstants.maxAngularRate, swerve.getGyroYaw()) ; 

    swerve.setControl(drive.withSpeeds(chassisSpeeds));
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
