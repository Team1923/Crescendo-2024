package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.CurrentConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AutoCommand.AutoScoreCommandGroup;
import frc.robot.commands.scoring.GCScoreCommandGroup;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private boolean currentLimitsActivated = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        setSwerveDriveCustomCurrentLimits();
        if(!currentLimitsActivated) {
            try {
                throw new Exception("Swerve Current Limits Not Active!");
            } catch (Exception e) {
                return;
            }
        }
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        setSwerveDriveCustomCurrentLimits();
        if(!currentLimitsActivated) {
            try {
                throw new Exception("Swerve Current Limits Not Active!");
            } catch (Exception e) {
                return;
            }
        }
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        SmartDashboard.putString("Request is being run", requestSupplier.get().toString());
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void setSwerveDriveCustomCurrentLimits() {   
        currentLimitsActivated = true;
        //Create a current configuration to use for the drive motor of each swerve module.
        var customCurrentLimitConfigs = new CurrentLimitsConfigs();

        //Iterate through each module.
        for (var module : Modules) {
            //Get the Configurator for the current drive motor.
            var currentConfigurator = module.getDriveMotor().getConfigurator();

            //Refresh the current configuration, since the stator current limit has already been set.
            currentConfigurator.refresh(customCurrentLimitConfigs);

            //Set all of the parameters related to the supply current.  The values should come from Constants.

            customCurrentLimitConfigs.StatorCurrentLimit = TunerConstants.kSwerveDriveStatorCurrentLimit;
            customCurrentLimitConfigs.StatorCurrentLimitEnable = TunerConstants.kSwerveStatorCurrentLimitEnable;

            // customCurrentLimitConfigs.SupplyCurrentLimit = SwerveConstants.kSwerveDriveSupplyCurrentLimit;
            // customCurrentLimitConfigs.SupplyCurrentLimitEnable = SwerveConstants.kSwerveDriveSupplyCurrentLimitEnable;
            // customCurrentLimitConfigs.SupplyCurrentThreshold = SwerveConstants.kSwerveDriveSupplyCurrentThreshold;
            // customCurrentLimitConfigs.SupplyTimeThreshold = SwerveConstants.kSwerveDriveSupplyTimeThreshold;

  

            //Apply the new current limit configuration.
            currentConfigurator.apply(customCurrentLimitConfigs);
        }
    }

    private void configurePathPlanner() {

        /*Have to do this here, because this needs to happen AFTER swerve subsystem is instantiated but BEFORE the autobuilder */
        NamedCommands.registerCommand("ScoreCommandGroup", new AutoScoreCommandGroup(this));
        NamedCommands.registerCommand("NonAutoScoreCommandGroup", new GCScoreCommandGroup(this, () -> 0, () -> 0, () -> 0));



        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            () -> this.getState().Pose,
            this::seedFieldRelative,
            this::getCurrentRobotChassisSpeeds,
            (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), //TODO: TUNE!
                new PIDConstants(5.0, 0, 0), //TODO: TUNE!
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius,
            new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                    },
            this);
    }  

    //getting from the pigeon used to generate CommandSwerveDriveTrain
    public Rotation2d getGyroYaw(){
        return Rotation2d.fromDegrees(Math.IEEEremainder(this.getPigeon2().getYaw().getValueAsDouble(),360));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }


    public void checkCurrentLimits(){

        int id = 0;
        for (var mod : Modules){

            var drive = mod.getDriveMotor();
            var steer = mod.getSteerMotor();

            if (Math.abs(drive.getStatorCurrent().getValueAsDouble())>(10+CurrentConstants.kStatorCurrentLimit)){
                SmartDashboard.putNumber("Over Stator on drive "+id, drive.getStatorCurrent().getValueAsDouble());
            }

            if (Math.abs(drive.getStatorCurrent().getValueAsDouble())>(10+CurrentConstants.kStatorCurrentLimit)){
                SmartDashboard.putNumber("Over Stator on steer "+id, steer.getStatorCurrent().getValueAsDouble());
            }

            id++;
        
        }
    }
    //getting from the pigeon used to generate CommandSwerveDriveTrain
    public void zeroGyro(){
        this.getPigeon2().setYaw(0);
    }
}
