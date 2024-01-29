// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.lib.LimelightUtil.LimelightInterface;
import frc.lib.RobotStateUtils.StateHandler;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.GroupLayout.Alignment;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToAmp extends Command {

    // private final double kP = 0.002; // TODO: TUNE
    // private final double rotationDeadband = 0.001; // TODO: tune

    PIDController rotationController = new PIDController(0.001, 0, 0);
    PIDController translationController = new PIDController(0.03, 0, 0);

    LimelightInterface limelight = LimelightInterface.getInstance();

    private SlewRateLimiter rotateLimiter;


    private SwerveSubsystem s_Swerve;
    // private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    // private BooleanSupplier robotCentricSup;
    // // private BooleanSupplier fieldSlowMode;

    // private PIDController pid;

    // private double angleGoal;

    // public AlignToAmp(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
    //         BooleanSupplier robotCentricSup/* , BooleanSupplier fieldSlowMode */) {
    //     this.s_Swerve = s_Swerve;
    //     addRequirements(s_Swerve);

    //     this.translationSup = translationSup;
    //     this.strafeSup = strafeSup;
    //     this.robotCentricSup = robotCentricSup;
    //     // this.fieldSlowMode = fieldSlowMode;

    //     // gives the angle of the amp relative to robot starting depending on alliance
    //     angleGoal = (DriverStation.getAlliance().get() == Alliance.Blue) ? 90 : -90;

    //     // TODO: tune kp
    //     pid = new PIDController(kP, 0, 0);

    //     pid.enableContinuousInput(-180, 180); // wraps around, treating -180 and 180 as same point

    // }

    public AlignToAmp(SwerveSubsystem s, DoubleSupplier strafeS){
        this.s_Swerve = s;
        this.strafeSup = strafeS;

        this.rotateLimiter = new SlewRateLimiter(Swerve.maxAngularAccel);


    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */

        // double rotationPercent = pid.calculate(s_Swerve.getGyroYaw().getDegrees(), angleGoal);

        // rotationPercent = (Math.abs(rotationPercent) > rotationDeadband) ? rotationPercent : 0;

        // SmartDashboard.putNumber("PID OUTPUT", rotationPercent);

        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Swerve.stickDeadband);
        double translationVal = MathUtil.applyDeadband(translationController.calculate(limelight.getXAngleOffset(), 0), 0.01);
        // double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Swerve.stickDeadband);
        // double rotationVal = MathUtil.applyDeadband(rotationPercent, Swerve.stickDeadband);

        //TODO: swap 90 and -90, blue should actually be -90, only using 90 because of where we put the apriltag in the hallway
        double rotationVal = rotateLimiter.calculate(rotationController.calculate(s_Swerve.getGyroYaw().getDegrees(), DriverStation.getAlliance().get() == Alliance.Blue ? 90 : -90)
);
        SmartDashboard.putNumber("ROTATION VAL", rotationVal);

        // if (DriverStation.getAlliance().get() == Alliance.Blue) {
        //     s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        //             rotationVal * Constants.Swerve.maxAngularVelocity, !robotCentricSup.getAsBoolean(), true);
        // } else {
        //     s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(-Constants.Swerve.maxSpeed),
        //             rotationVal * Constants.Swerve.maxAngularVelocity, !robotCentricSup.getAsBoolean(), true);
        // }


        s_Swerve.drive(new Translation2d(strafeVal, -translationVal).times(Constants.Swerve.maxSpeed), 
        rotationVal * Constants.Swerve.maxAngularVelocity, 
        false, true);

    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}