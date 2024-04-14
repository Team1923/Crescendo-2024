// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.Limelight.LimelightInterface;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/* TODO: cross check with existing code to get state handler integration going */
public class GoalCentricCommandRequest extends Command {
  
  /* Swerve + Limelight Initialization */
  private CommandSwerveDrivetrain swerve;
  private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() 
  .withDeadband(SwerveConstants.maxSpeed * 0.1).withRotationalDeadband(SwerveConstants.maxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private StateHandler stateHandler = StateHandler.getInstance();

  /* Fancy double stuff for input + output to swerve. */
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  /* PID Things */
  private PIDController rotationController;

  /** Creates a new GoalCentricCommand. */
  public GoalCentricCommandRequest(CommandSwerveDrivetrain swerve, DoubleSupplier t, DoubleSupplier s, DoubleSupplier r) {
    this.swerve = swerve;
    this.translationSup = t;
    this.strafeSup = s;
    this.rotationSup = r;

    rotationController = new PIDController(Constants.SwerveConstants.headingKP, Constants.SwerveConstants.headingKI, Constants.SwerveConstants.headingKD);
    addRequirements(this.swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setIsGoalCentric(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationValue = Math.abs(translationSup.getAsDouble()) > 0.1 ? translationSup.getAsDouble() : 0;
    double strafeValue = Math.abs(strafeSup.getAsDouble()) > 0.1 ? strafeSup.getAsDouble() : 0;
    double rotValue = 0;
    if  (stateHandler.getScoreInAmp() || stateHandler.getScoreInSubwoofer() || stateHandler.getScoreInReverseSubwoofer() || stateHandler.getScoreInTrap()){      //babyproofing from misclick
      rotValue = MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.1);
    }
    else if(Math.abs(rotationSup.getAsDouble()) > 0.5){
      rotValue = rotationSup.getAsDouble();
    } else if(stateHandler.getHasValidSpeakerTag()){
      rotValue = rotationController.calculate(stateHandler.getxAngleOffset(), 0); 
    } else{
      rotValue = 0;
    }

    // SmartDashboard.putNumber("ROT VAL", rotValue);

      swerve.setControl(drive.withVelocityX(translationValue)
      .withVelocityY(strafeValue)
      .withRotationalRate(rotValue));
    

    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setIsGoalCentric(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//stateHandler.getWantToPositionForSubwoofer() || stateHandler.getScoreInAmp() || stateHandler.getReverseSubwoofer();
  }
}
