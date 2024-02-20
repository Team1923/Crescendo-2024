package frc.robot.subsystems;

import frc.lib.SwerveUtil.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    /* Odometry object used to keep track of the robot's position on the field. */
    public SwerveDriveOdometry swerveOdometry;

    /* Array of Swerve Modules. 
     * Each module contains:
     * - Drive Motor
     * - Steering Motor
     * - CANCoder (absolute encoder)
     */
    public SwerveModule[] mSwerveMods;

    /* Gyro/Accelerometer declaration. */
    public Pigeon2 gyro;

    public SwerveSubsystem() {
        /* Initialize gyro, wipe configuration, and reset angle to 0 degrees. */
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "rio");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        /* Initialize array of swerve modules. Assign a module number to each module. */
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* Delay needed to properly ensure that modules are reset to their absolute position. */
        Timer.delay(1.5);
        resetModulesToAbsolute();

        /* Initialize SwerveOdometry object. */
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        /* PathPlanner configuration. */
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::setPose, 
            this::getRobotRelativeSpeeds, 
            this::driveRobotRelativeForPP, 
            Constants.Swerve.pathFollowerConfig, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);
    }

    /* Method to set the module states & drive the robot. */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds fieldChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), 
            rotation, getHeading());

        ChassisSpeeds robotRelChassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.discretize(fieldChassisSpeeds, 0.02) 
                : ChassisSpeeds.discretize(robotRelChassisSpeeds, 0.02));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);        
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }  
    
    /* Drive ROBOT RELATIVE - only for PathPlanner! */
    public void driveRobotRelativeForPP(ChassisSpeeds robotRelativeSpeeds) {
        SwerveModuleState[] swerveModuleStates = 
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02));
        setModuleStates(swerveModuleStates);
    }

    /* Get the current velocity of the modules relative to the robot's motion. */
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /* Get the current state of the modules (speed, position, direction, etc.) */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /* Get the specific position of the modules. */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /* Get the pose of the robot returned by the odometry class. */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /* Set the pose that the odometry class references. */
    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    /* Get the heading of the robot. */
    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    /* Set/override the heading of the robot. */
    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    /**
     * Resets the robots heading to zero.
     */
    public void zeroGyro(){
        gyro.setYaw(0);
    }

    /* Reset the odometry */
    public void resetOdometry(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /* Gets the gyro's current yaw using IEEE remainder. Needed to fulfill range of (-180, 180]*/
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getYaw().getValue(), 360));
    }

    /* Reset all the swerve modules. */
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    /* Stop the modules (pass in a velocity of 0) */
    public void stop() {
        drive(new Translation2d(0, 0), 0, true, false);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        PathPlannerLogging.logCurrentPose(getPose());

        SmartDashboard.putNumber("HEADING", getGyroYaw().getDegrees());



        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}