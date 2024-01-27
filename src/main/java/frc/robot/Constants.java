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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.SwerveUtil.COTSTalonFXSwerveConstants;
import frc.lib.SwerveUtil.SwerveModuleConstants;

public final class Constants {

    public static final class LimeLightConstants{
        //measure constants in inches
        public static final double limelightMountAngle = 11.5; // for pivot, measured in degrees
        public static final double limelightHeight = 10.5625; //for pivot, in inches (measured ~10 and 9/16 inches, converted)
        public static final double speakerHeightFromFloor = 59.25;//for tag, measured
        public static final double limelightViewingAngle = 28.9; //degrees

    }

    public static final class IntakeConstants {
        /* Motor IDs */
        public static final int intakeRollerMotorID = 0;
        public static final int intakePrimaryID = 0;
        public static final int intakeFollowerID = 0;
    
        /* Motion Magic Constants */
        public static final double intakekP = 0;
        public static final double intakekI = 0;
        public static final double intakekD = 0;
        public static final double maxIntakeVel = 0;
        public static final double maxIntakeAccel = 0;
    
        /* Gearbox Ratios & Unit Conversions */
        public static final double intakeGearRatio = 0;
        //TODO: check this math? compared it against the 2023 repo and plugged into phoenix converter...
        public static final double intakeRotsToRads = (2 * Math.PI) / intakeGearRatio;
        public static final double intakeRadsToRots = intakeGearRatio / (2 * Math.PI);
    
        /* kG - gravity constant for motion of arm */
        //TODO: discuss on how the max gravity constant should be found.
        public static final double intakeMaxGravityConstant = 0;

        /* Beam Break ID */
        public static final int beamBreakOneID = 0;
    }

    public static final class ShooterConstants {
        /* Motor IDs */
        public static final int shooterMotorPrimaryID = 0;
        public static final int shooterMotorFollowerID = 0;
    
        /* Beam Break ID */
        public static final int beamBreakFourID = 0;
      }
    
      public static final class FeederConstants {
        /* Motor ID */
        public static final int feederMotorID = 0;
    
        /* Beam Break IDs */
        public static final int beamBreakTwoID = 0;
        public static final int beamBreakThreeID = 0;
    }

    public static final class ArmConstants {
        /* Motor IDs */
        public static final int armMotorPrimaryID = 0;
        public static final int armMotorFollowerID = 0;
    
        /* Motion Magic Constants */
        public static final double armkP = 0;
        public static final double armkI = 0;
        public static final double armkD = 0;
        public static final double maxArmVel = 0;
        public static final double maxArmAccel = 0;
    
        /* Gearbox Ratios & Unit Conversions */
        public static final double armGearRatio = 0;
        //TODO: check this math? compared it against the 2023 repo and plugged into phoenix converter...
        public static final double armRotsToRads = (2 * Math.PI) / armGearRatio;
        public static final double armRadsToRots = armGearRatio / (2 * Math.PI);
    
        /* kG - gravity constant for motion of arm */
        //TODO: discuss on how the max gravity constant should be found.
        public static final double armMaxGravityConstant = 0;  
    }


    public static final class Swerve {
        public static final double stickDeadband = 0.1;
        public static final int pigeonID = 13;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.75); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(18.75); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
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

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.4864; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 35; //TODO: This must be tuned to specific robot

        /* Acceleration Parameters - CHANGE IF NEEDED*/
        public static final double maxAccel = 3;
        public static final double maxAngularAccel = 3;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(95.009);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(156.97);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(110.023);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-54.755);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final HolonomicPathFollowerConfig pathFollowerConfig = 
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), //might need to lower P value
                new PIDConstants(0.3, 0, 0), maxSpeed, 
                Math.hypot(trackWidth / 2, wheelBase / 2),
                new ReplanningConfig());
    }
}
