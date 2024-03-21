package frc.robot.lib.ShooterArmUtils;

import java.util.HashMap;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.lib.StateMachine.StateVariables;
import frc.robot.lib.StateMachine.StateVariables.ArmPosition;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;
import frc.robot.lib.StateMachine.StateVariables.ShooterSpeeds;

public class PositionRPMData {
    private static PositionRPMData positionRPMData;
    private static HashMap<Double, PositionRPMObject> speakerPositionRPMMap = new HashMap<>();
    private static HashMap<Double, PositionRPMObject> trapPositionRPMMap = new HashMap<>();

    
    public static synchronized PositionRPMData getInstance() {
        if (positionRPMData == null) {
            positionRPMData = new PositionRPMData();
        }

        return positionRPMData;
     }

     /**
      * Creates an instance of PositionRPMData & fills up HashMap with data.
      */
     public PositionRPMData() {
        fillMaps();
     }

     private void fillMaps() {
        /*New Data */
        speakerPositionRPMMap.put(52.43, new PositionRPMObject(-0.65 + ArmConstants.armPositionChange, 2500));
        speakerPositionRPMMap.put(59.95, new PositionRPMObject(-0.61 + ArmConstants.armPositionChange, 2500));
        speakerPositionRPMMap.put(68.55, new PositionRPMObject(-0.57 + ArmConstants.armPositionChange, 2500));
        speakerPositionRPMMap.put(75.78, new PositionRPMObject(-0.49 + ArmConstants.armPositionChange, 2700));
        speakerPositionRPMMap.put(84.53, new PositionRPMObject(-0.45 + ArmConstants.armPositionChange, 2700));
        speakerPositionRPMMap.put(92.3, new PositionRPMObject(-0.41 + ArmConstants.armPositionChange, 2700));
        speakerPositionRPMMap.put(98.9, new PositionRPMObject(-0.395 + ArmConstants.armPositionChange, 2800));
        speakerPositionRPMMap.put(106.54, new PositionRPMObject(-0.37 + ArmConstants.armPositionChange, 2800));
        speakerPositionRPMMap.put(113.13, new PositionRPMObject(-0.335 + ArmConstants.armPositionChange, 2800));
        speakerPositionRPMMap.put(120.5, new PositionRPMObject(-0.31 + ArmConstants.armPositionChange, 2900));
        speakerPositionRPMMap.put(127.1, new PositionRPMObject(-0.2925 + ArmConstants.armPositionChange, 3200));
        speakerPositionRPMMap.put(132.47, new PositionRPMObject(-0.2775 + ArmConstants.armPositionChange, 3200));
        speakerPositionRPMMap.put(139.45, new PositionRPMObject(-0.255 + ArmConstants.armPositionChange, 3300));
        speakerPositionRPMMap.put(145.6, new PositionRPMObject(-0.245 + ArmConstants.armPositionChange, 3500));
        speakerPositionRPMMap.put(151.5, new PositionRPMObject(-0.225 + ArmConstants.armPositionChange, 3500));
        speakerPositionRPMMap.put(156.4, new PositionRPMObject(-0.215 + ArmConstants.armPositionChange, 3500));
        speakerPositionRPMMap.put(161.7, new PositionRPMObject(-0.2 + ArmConstants.armPositionChange, 3650));
        speakerPositionRPMMap.put(167.6, new PositionRPMObject(-0.1975 + ArmConstants.armPositionChange, 3650));
        speakerPositionRPMMap.put(172.1, new PositionRPMObject(-0.1875 + ArmConstants.armPositionChange, 3700));
        speakerPositionRPMMap.put(176.3, new PositionRPMObject(-0.17 + 1.2*ArmConstants.armPositionChange, 3750));
        speakerPositionRPMMap.put(182.6, new PositionRPMObject(-0.165 + 1.2*ArmConstants.armPositionChange, 3825));
        speakerPositionRPMMap.put(186.6 , new PositionRPMObject(-0.155 + 1.2*ArmConstants.armPositionChange, 4000));
        speakerPositionRPMMap.put(190.4, new PositionRPMObject(-0.135 + ArmConstants.armPositionChange, 4200));
        speakerPositionRPMMap.put(194.9, new PositionRPMObject(-0.13 + ArmConstants.armPositionChange, 4200));
        speakerPositionRPMMap.put(197.7, new PositionRPMObject(-0.115 + ArmConstants.armPositionChange, 4350));
      
     }

     /**
      * Method to get the desired arm position based on Speaker distance.
      * @param distance the limelight's measured distance from Speaker.
      * @return a setpoint for the arm's position.
      */
     public double getSpeakerDesiredArmPosition(double distance) {
        double lowerBound = LimeLightConstants.speakerLerpLowerBound;
        double upperBound = LimeLightConstants.speakerLerpUpperBound;

        if (distance < lowerBound) {
            return ArmStates.STOWED.getArmPosition().getAngularSetpoint();
        } else if (distance > upperBound) {
            return ArmStates.STOWED.getArmPosition().getAngularSetpoint();
        }

        Double[] distanceValues = (Double[]) speakerPositionRPMMap.keySet().toArray(new Double[speakerPositionRPMMap.size()]);

        for (int i = 0; i < distanceValues.length - 1; i++) {
            if (distance >= distanceValues[i] && distance <= distanceValues[i + 1]) {
                lowerBound = distanceValues[i];
                upperBound = distanceValues[i + 1];
            }
        }

        double deltaDistance = distance - lowerBound;
        double deltaPositionValues = (speakerPositionRPMMap.get(upperBound).getArmPosition()) - 
            (speakerPositionRPMMap.get(lowerBound).getArmPosition());
        double deltaDistanceBounds = upperBound - lowerBound;

        return (speakerPositionRPMMap.get(lowerBound).getArmPosition()) + 
            ((deltaDistance * deltaPositionValues) / deltaDistanceBounds);
     }

     /**
      * Method to get the desired shooter RPM based on Speaker distance.
      * @param distance the limelight's measured distance from the Speaker.
      * @return an RPM setpoint for the shooter's velocity.
      */
     public double getSpeakerDesiredShooterRPM(double distance) {
        double lowerBound = LimeLightConstants.speakerLerpLowerBound;
        double upperBound = LimeLightConstants.speakerLerpUpperBound;

        if (distance < lowerBound) {
            return ShooterSpeeds.IDLE.getRPMValue().getRPM();
        } else if (distance > upperBound) {
            return ShooterSpeeds.IDLE.getRPMValue().getRPM();
        }

        Double[] distanceValues = (Double[]) speakerPositionRPMMap.keySet().toArray(new Double[speakerPositionRPMMap.size()]);

        for (int i = 0; i < distanceValues.length - 1; i++) {
            if (distance >= distanceValues[i] && distance <= distanceValues[i + 1]) {
                lowerBound = distanceValues[i];
                upperBound = distanceValues[i + 1];
            }
        }

        double deltaDistance = distance - lowerBound;
        double deltaRPMValues = (speakerPositionRPMMap.get(upperBound).getShooterRPM()) - 
            (speakerPositionRPMMap.get(lowerBound).getShooterRPM());
        double deltaDistanceBounds = upperBound - lowerBound;

        return (speakerPositionRPMMap.get(lowerBound).getShooterRPM()) + ((deltaDistance * deltaRPMValues) / deltaDistanceBounds);
     }

     /**
      * Method to get the desired arm position based on Trap distance.
      * @param distance the limelight's measured distance from Trap.
      * @return a setpoint for the arm's position.
      */
     public double getTrapDesiredArmPosition(double distance) {
        double lowerBound = LimeLightConstants.trapLerpLowerBound;
        double upperBound = LimeLightConstants.trapLerpUpperBound;

        if (distance < lowerBound) {
            return ArmStates.STOWED.getArmPosition().getAngularSetpoint();
        } else if (distance > upperBound) {
            return ArmStates.STOWED.getArmPosition().getAngularSetpoint();
        }

        Double[] distanceValues = (Double[]) trapPositionRPMMap.keySet().toArray(new Double[trapPositionRPMMap.size()]);

        for (int i = 0; i < distanceValues.length - 1; i++) {
            if (distance >= distanceValues[i] && distance <= distanceValues[i + 1]) {
                lowerBound = distanceValues[i];
                upperBound = distanceValues[i + 1];
            }
        }

        double deltaDistance = distance - lowerBound;
        double deltaPositionValues = (trapPositionRPMMap.get(upperBound).getArmPosition()) - 
            (trapPositionRPMMap.get(lowerBound).getArmPosition());
        double deltaDistanceBounds = upperBound - lowerBound;

        return (trapPositionRPMMap.get(lowerBound).getArmPosition()) + ((deltaDistance * deltaPositionValues) / deltaDistanceBounds);
     }

     /**
      * Method to get the desired shooter RPM based on Trap distance.
      * @param distance the limelight's measured distance from the Trap.
      * @return an RPM setpoint for the shooter's velocity.
      */
     public double getTrapDesiredShooterRPM(double distance) {
        double lowerBound = LimeLightConstants.trapLerpLowerBound;
        double upperBound = LimeLightConstants.trapLerpUpperBound;

        if (distance < lowerBound) {
            return ShooterSpeeds.IDLE.getRPMValue().getRPM();
        } else if (distance > upperBound) {
            return ShooterSpeeds.IDLE.getRPMValue().getRPM();
        }

        Double[] distanceValues = (Double[]) trapPositionRPMMap.keySet().toArray(new Double[trapPositionRPMMap.size()]);

        for (int i = 0; i < distanceValues.length - 1; i++) {
            if (distance >= distanceValues[i] && distance <= distanceValues[i + 1]) {
                lowerBound = distanceValues[i];
                upperBound = distanceValues[i + 1];
            }
        }

        double deltaDistance = distance - lowerBound;
        double deltaRPMValues = (trapPositionRPMMap.get(upperBound).getShooterRPM()) - 
            (trapPositionRPMMap.get(lowerBound).getShooterRPM());
        double deltaDistanceBounds = upperBound - lowerBound;

        return (trapPositionRPMMap.get(lowerBound).getShooterRPM()) + ((deltaDistance * deltaRPMValues) / deltaDistanceBounds);
     }
}
