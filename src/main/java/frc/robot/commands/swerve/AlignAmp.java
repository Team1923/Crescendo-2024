// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;


import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;



import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignAmp extends Command {

   private SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final double kP = 0.002; // TODO: TUNE
    // private final double rotationDeadband = 0.001; // TODO: tune

    PIDController rotationController = new PIDController(SwerveConstants.headingKP, SwerveConstants.headingKI, SwerveConstants.headingKD);
    PIDController translationController = new PIDController(SwerveConstants.translationKP, SwerveConstants.translationKI, SwerveConstants.translationKD);


    private SlewRateLimiter rotateLimiter;


    private CommandSwerveDrivetrain s_Swerve;
    private DoubleSupplier strafeSup;
    

    public AlignAmp(CommandSwerveDrivetrain s, DoubleSupplier strafeS){
        this.s_Swerve = s;
        this.strafeSup = strafeS;

        rotationController.enableContinuousInput(-180, 180);




    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */

        

        double translationVal = MathUtil.applyDeadband(translationController.calculate(StateHandler.getInstance().getxAngleOffset(), 0), 0.01);

        
        double rotationVal = rotationController.calculate(s_Swerve.getGyroYaw().getDegrees(), (DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : -90);


      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-translationVal * SwerveConstants.maxSpeed, 
     strafeSup.getAsDouble()  * SwerveConstants.maxSpeed, 
      rotationVal * SwerveConstants.maxAngularRate, s_Swerve.getGyroYaw()); 
    
      s_Swerve.setControl(drive.withSpeeds(chassisSpeeds));

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}