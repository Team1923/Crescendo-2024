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

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToAmpWTranslate extends Command {

    // private final double kP = 0.002; // TODO: TUNE
    // private final double rotationDeadband = 0.001; // TODO: tune

    PIDController rotationController = new PIDController(0.001, 0, 0);
    PIDController translationController = new PIDController(0.03, 0, 0);

    StateHandler stateHandler = StateHandler.getInstance();
    LimelightInterface limelight = LimelightInterface.getInstance();

    private SlewRateLimiter rotateLimiter;

    private final double TOLERANCE = 2;


    private SwerveSubsystem s_Swerve;
    private DoubleSupplier translationSup;
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

    public AlignToAmpWTranslate(SwerveSubsystem s, DoubleSupplier strafeS, DoubleSupplier transS){
        this.s_Swerve = s;
        this.strafeSup = strafeS;
        this.translationSup = transS;

        this.rotateLimiter = new SlewRateLimiter(Swerve.maxAngularAccel);


        addRequirements(s_Swerve);


    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        System.out.println("Working");
        /* Get Values, Deadband */

        // double rotationPercent = pid.calculate(s_Swerve.getGyroYaw().getDegrees(), angleGoal);

        // rotationPercent = (Math.abs(rotationPercent) > rotationDeadband) ? rotationPercent : 0;

        // SmartDashboard.putNumber("PID OUTPUT", rotationPercent);

        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Swerve.stickDeadband);
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Swerve.stickDeadband);

        //-GM: I know this doesn't adapt to the +-90 depending on alliance, but that shouldn't matter since if you are the wrong way you would not have the valid amp tag?
        if (stateHandler.getHasValidAmpTag() && Math.abs(90-Math.abs(s_Swerve.getGyroYaw().getDegrees())) < TOLERANCE){
            translationVal = MathUtil.applyDeadband(translationController.calculate(limelight.getXAngleOffset(), 0), 0.01);
        }

        //TODO: swap 90 and -90, blue should actually be -90, only using 90 because of where we put the apriltag in the hallway fixed code: 

        double rotationVal = rotateLimiter.calculate(rotationController.calculate(s_Swerve.getGyroYaw().getDegrees(), DriverStation.getAlliance().get() == Alliance.Blue ? -90 : 90)

);
        // SmartDashboard.putNumber("ROTATION VAL", rotationVal);

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                    rotationVal * Constants.Swerve.maxAngularVelocity, true, true);
        } else {
            s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(-Constants.Swerve.maxSpeed),
                    rotationVal * Constants.Swerve.maxAngularVelocity, true, true);
        }


        // s_Swerve.drive(new Translation2d(strafeVal, -translationVal).times(Constants.Swerve.maxSpeed), 
        // rotationVal * Constants.Swerve.maxAngularVelocity, 
        // false, true);

    }

    @Override
    public void end(boolean interrupted) {
        // s_Swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}