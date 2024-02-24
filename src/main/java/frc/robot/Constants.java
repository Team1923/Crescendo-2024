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

    public static final class SwerveConstants {
        public static final double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        public static final double maxAngularRate = (1.5 * Math.PI) * 8;
    }

    public static final class LimeLightConstants {
        // measure constants in inches
        public static final double limelightMountAngle = 27.358; // for pivot, measured in degrees
        public static final double limelightHeight = 7; // for pivot, in inches (measured ~10 and 9/16 inches,
                                                              // converted)
        public static final double speakerHeightFromFloor = 56.5;// for tag, measured
        public static final double xAngleThreshold = 2;
        public static final double lerpLowerBound = 84;
        public static final double lerpUpperBound = 168;
    }

    public static final class ControllerConstants {
        
    }

}