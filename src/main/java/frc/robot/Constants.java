// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class CurrentConstants{
        public static final double kStatorCurrentLimit = 80;
        public static final boolean kStatorCurrentLimitEnable = true;
    }

    public static final class SwerveConstants {
        public static final double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        public static final double maxAngularRate = (1.5 * Math.PI) * 8;
    }

    public static final class LimeLightConstants {
        // measure constants in inches
        /*
         * Dimensions for LL tower
         * Front LL height -> 26.85 inches 
         * Front LL Mount angle -> 20 degrees
         * 
         */
        public static final double limelightMountAngle = 27.358; // for pivot, measured in degrees
        public static final double limelightHeight = 7; // for pivot, in inches (measured ~10 and 9/16 inches,
                                                              // converted)
        public static final double speakerHeightFromFloor = 56.5;// for tag, measured
        public static final double speakerLerpLowerBound = 52.43;
        public static final double speakerLerpUpperBound = 197.7;

        //TODO: TUNE
        public static final double trapHeightFromFloor = 0; 
        public static final double trapLerpLowerBound = 0;
        public static final double trapLerpUpperBound = 0;
        

        public static final double xAngleThreshold = 2.5;
        
    }

    public static final class ControllerConstants {

        static final class Driver{
            public static final int yButton = 4;
            public static final int aButton = 1;
            public static final int xButton = 3;
            public static final int bButton = 2;

            public static final int driverLeftBumper = 5;
            public static final int driverRightBumper = 6;
        }

        static final class Operator{
            public static final int triangleButton = 4;
            public static final int squareButton = 1; //1 for PS5, 3 for PS4
            public static final int circleButton = 3; //3 for PS5, 2 for PS4
            public static final int crossButton = 2; // 2 for PS5, 1 for PS4

            public static final int littleRightButton = 10; //10 for PS5, 8 for PS4
            public static final int littleLeftButton = 9; //9 for PS5, 7 for PS4

            public static final int operatorLeftBumper = 5;
            public static final int operatorRightBumper = 6;
            public static final int operatorRightTrigger = 8;
        }
        
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
        public static final double intakekV = 0.1;
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
        public static final double maxIntakeVel = 100;
        public static final double maxIntakeAccel = 500;
        public static final double maxIntakeJerk = 2200;

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
        public static final double shooterKV = 0.115; //0.12
        public static final double shooterKP = 0.2;
        public static final double shooterKA = 0;
        public static final double shooterKI = 0;
        public static final double shooterKD = 0;
        public static final double maxShooterAccel = 400;
        public static final double maxShooterJerk = 5000;

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
        public static final double armkP = 1.2;
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
        public static final double armMaxGravityConstant = 0.03 * 12; // 2 volts max ff 

        public static final double armPositionAllowableOffset = 0.03; // allowed radians offset

        public static final double armPositionChange = -0.044;
    }

    public static final class LEDConstants{
        public static final int LEDCount = 41 + 8;
        public static final int CANdleID = 23;


    }

    public static final class ButtonConstants{
        public static final int insideButton = 5;
        public static final int outsideButton = 6;

    }

}