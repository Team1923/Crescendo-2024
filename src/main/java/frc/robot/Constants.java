package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.SwerveUtil.COTSTalonFXSwerveConstants;
import frc.lib.SwerveUtil.SwerveModuleConstants;

public final class Constants {

    public static final class LimeLightConstants {
        // measure constants in inches
        public static final double limelightMountAngle = 11.5; // for pivot, measured in degrees
        public static final double limelightHeight = 10.5625; // for pivot, in inches (measured ~10 and 9/16 inches,
                                                              // converted)
        public static final double speakerHeightFromFloor = 59.25;// for tag, measured
        public static final double limelightViewingAngle = 28.9; // degrees

    }

    public static final class IntakeConstants {
        /* Motor IDs */
        public static final int intakeWheelTopID = 25; //NOT ON ROBOT
        public static final int intakeWheelBottomID = 26; //NOT ON ROBOT
        public static final int intakeArmPrimaryID = 22; //left
        public static final int intakeArmFollowerID = 21; //right

        /* Motion Magic Constants */
        public static final double intakeKS = 0;
        public static final double intakekP = 0.9;
        public static final double intakekI = 0.005;
        public static final double intakekD = 0;

        /*
        * Values from CTRE
        * motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
        * motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration
        * (0.5 seconds)
        * motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
        */
        
        //note, these should be VERY slow
        public static final double maxIntakeVel = 80;
        public static final double maxIntakeAccel = 160;
        public static final double maxIntakeJerk = 1600;

        /* Gearbox Ratios & Unit Conversions */
        public static final double intakeGearRatio = 60;
        public static final double intakeRotsToRads = (2 * Math.PI) / intakeGearRatio;
        public static final double intakeRadsToRots = intakeGearRatio / (2 * Math.PI);

        /* kG - gravity constant for motion of arm */
        // TODO: discuss on how the max gravity constant should be found.
        public static final double intakeMaxGravityConstant = 0;

        /* Beam Break ID */
        public static final int beamBreakOneID = 1;

        public static final double intakePositionAllowableOffset = 0.05; //radians
    }

    public static final class ShooterConstants {
        /* Motor IDs */
        public static final int shooterMotorPrimaryID = 17; //top
        public static final int shooterMotorFollowerID = 18; //bottom 

        public static final double shooterRPSToRPM = 60;
        public static final double shooterRPMToRPS = 1 / shooterRPSToRPM;

        /* Motion Magic Velocity Constants */
        public static final double shooterKS = 0.25;
        public static final double shooterKV = 0.12;
        public static final double shooterKP = 0.5;
        public static final double shooterKA = 0.01;
        public static final double shooterKI = 0.001;
        public static final double shooterKD = 0;
        public static final double maxArmAccel = 100;
        public static final double maxArmJerk = 1000;

        /* Beam Break ID */
        public static final int beamBreakFourID = 4;

        public static final double shooterSpeedThreshold = 50; //RPM


    }

    public static final class FeederConstants {
        /* Motor ID */
        public static final int feederMotorID = 14;

        /* Beam Break IDs */
        public static final int beamBreakTwoID = 2;
        public static final int beamBreakThreeID = 3;
    }

    public static final class ArmConstants {
        /* Motor IDs */
        public static final int armMotorPrimaryID = 15; //right
        public static final int armMotorFollowerID = 16; //left

        /* Motion Magic Constants */
        public static final double armKS = 0;
        public static final double armkP = 0.8;
        public static final double armkI = 0.005;
        public static final double armkD = 0;
        public static final double maxArmVel = 100;
        public static final double maxArmAccel = 250;
        public static final double maxArmJerk = 1000;

        /* Gearbox Ratios & Unit Conversions */
        public static final double armGearRatio = 129.6;
        public static final double armRotsToRads = (2 * Math.PI) / armGearRatio;
        public static final double armRadsToRots = armGearRatio / (2 * Math.PI);

        /* kG - gravity constant for motion of arm */
        // TODO: discuss on how the max gravity constant should be found.
        public static final double armMaxGravityConstant = 0.045 * 12;

        public static final double armPositionAllowableOffset = 0.05; // allowed radians offset

        public static final double SUBWOOFER_THRESHHOLD = 0; // TODO: tune, how far away we are from subwoofer before we move
                                                             // arm to subwoofer pos.
    }

    public static final class Swerve {
        public static final double stickDeadband = 0.1;
        public static final int pigeonID = 13;

        public static final COTSTalonFXSwerveConstants chosenModule =
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.75);
        public static final double wheelBase = Units.inchesToMeters(18.75); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
        * These values are used by the drive falcon to ramp in open loop and closed
        * loop driving.
        * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
        */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; 
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.4864; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 40;

        /* Acceleration Parameters - CHANGE IF NEEDED */
        public static final double maxAccel = 3;
        public static final double maxAngularAccel = 3;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(143.375); //TODO: tune
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(89.824); //TODO: tune
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(127); //TODO: tune
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(137.46); //TODO: tune
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // might need to lower P value
                new PIDConstants(0.3, 0, 0), maxSpeed,
                Math.hypot(trackWidth / 2, wheelBase / 2),
                new ReplanningConfig());
    }
}
