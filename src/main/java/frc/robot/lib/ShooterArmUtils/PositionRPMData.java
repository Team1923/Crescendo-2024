package frc.robot.lib.ShooterArmUtils;

import java.util.HashMap;

import frc.robot.Constants.LimeLightConstants;

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

        /* MANUAL DATA */
        // speakerPositionRPMMap.put(149.25, new PositionRPMObject(-0.285, 4000));
        // speakerPositionRPMMap.put(153.1, new PositionRPMObject(-0.285, 4000));
        // speakerPositionRPMMap.put(103.6, new PositionRPMObject(-0.43, 3000));
        // speakerPositionRPMMap.put(137.0, new PositionRPMObject(-0.32, 4000));
        // speakerPositionRPMMap.put(222.7, new PositionRPMObject(0, 4200));
        // speakerPositionRPMMap.put(105.1, new PositionRPMObject(-0.39, 3000));
        // speakerPositionRPMMap.put(235.9, new PositionRPMObject(0, 4500));
        // speakerPositionRPMMap.put(231.6, new PositionRPMObject(0, 4500));
        // speakerPositionRPMMap.put(198.2, new PositionRPMObject(-0.195, 4200));
        // speakerPositionRPMMap.put(181.5, new PositionRPMObject(-0.215, 4200));
        // speakerPositionRPMMap.put(159.4, new PositionRPMObject(-0.235, 4000));
        // speakerPositionRPMMap.put(92.1, new PositionRPMObject(-0.48, 3000));
        // speakerPositionRPMMap.put(78.7, new PositionRPMObject(-0.53, 3000));
        // speakerPositionRPMMap.put(56.6, new PositionRPMObject(-0.63, 3000));
        // speakerPositionRPMMap.put(52.3, new PositionRPMObject(-0.68, 3000));
        // speakerPositionRPMMap.put(117.6, new PositionRPMObject(-0.34, 3300));
        // speakerPositionRPMMap.put(126.3, new PositionRPMObject(-0.35, 3300));


        /*New Data */
        speakerPositionRPMMap.put(52.43, new PositionRPMObject(-0.65, 2500));
        speakerPositionRPMMap.put(59.95, new PositionRPMObject(-0.61, 2500));
        speakerPositionRPMMap.put(68.55, new PositionRPMObject(-0.57, 2500));
        speakerPositionRPMMap.put(75.78, new PositionRPMObject(-0.49, 2700));
        speakerPositionRPMMap.put(84.53, new PositionRPMObject(-0.45, 2700));
        speakerPositionRPMMap.put(92.3, new PositionRPMObject(-0.41, 2700));
        speakerPositionRPMMap.put(98.9, new PositionRPMObject(-0.395, 2800));
        speakerPositionRPMMap.put(106.54, new PositionRPMObject(-0.37, 2800));
        speakerPositionRPMMap.put(113.13, new PositionRPMObject(-0.335, 2800));
        speakerPositionRPMMap.put(120.5, new PositionRPMObject(-0.31, 2900));
        speakerPositionRPMMap.put(127.1, new PositionRPMObject(-0.2925, 3200));
        speakerPositionRPMMap.put(132.47, new PositionRPMObject(-0.2775, 3200));
        speakerPositionRPMMap.put(139.45, new PositionRPMObject(-0.255, 3300));
        speakerPositionRPMMap.put(145.6, new PositionRPMObject(-0.245, 3500));
        speakerPositionRPMMap.put(151.5, new PositionRPMObject(-0.225, 3500));
        speakerPositionRPMMap.put(156.4, new PositionRPMObject(-0.215, 3500));
        speakerPositionRPMMap.put(161.7, new PositionRPMObject(-0.2, 3650));
        speakerPositionRPMMap.put(167.6, new PositionRPMObject(-0.1975, 3650));
        speakerPositionRPMMap.put(172.1, new PositionRPMObject(-0.1875, 3700));
        speakerPositionRPMMap.put(176.3, new PositionRPMObject(-0.17, 3750));
        speakerPositionRPMMap.put(182.6, new PositionRPMObject(-0.165, 3825));
        speakerPositionRPMMap.put(186.6 , new PositionRPMObject(-0.155, 4000));
        speakerPositionRPMMap.put(190.4, new PositionRPMObject(-0.135, 4200));
        speakerPositionRPMMap.put(194.9, new PositionRPMObject(-0.13, 4200));
        speakerPositionRPMMap.put(197.7, new PositionRPMObject(-0.115, 4350));
      
       






        // trapPositionRPMMap.put(null, null);
     }

     /**
      * Method to get the desired arm position based on Speaker distance.
      * @param distance the limelight's measured distance from Speaker.
      * @return a setpoint for the arm's position.
      */
     public double getSpeakerDesiredArmPosition(double distance) {
        double lowerBound = LimeLightConstants.speakerLerpLowerBound;
        double upperBound = LimeLightConstants.speakerLerpUpperBound;

        if (distance <= lowerBound) {
            return speakerPositionRPMMap.get(lowerBound).getArmPosition();
        } else if (distance >= upperBound) {
            return speakerPositionRPMMap.get(upperBound).getArmPosition();
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

        return (speakerPositionRPMMap.get(lowerBound).getArmPosition()) + ((deltaDistance * deltaPositionValues) / deltaDistanceBounds);
     }

     /**
      * Method to get the desired shooter RPM based on Speaker distance.
      * @param distance the limelight's measured distance from the Speaker.
      * @return an RPM setpoint for the shooter's velocity.
      */
     public double getSpeakerDesiredShooterRPM(double distance) {
        double lowerBound = LimeLightConstants.speakerLerpLowerBound;
        double upperBound = LimeLightConstants.speakerLerpUpperBound;

        if (distance <= lowerBound) {
            return speakerPositionRPMMap.get(lowerBound).getShooterRPM();
        } else if (distance >= upperBound) {
            return speakerPositionRPMMap.get(upperBound).getShooterRPM();
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

        if (distance <= lowerBound) {
            return trapPositionRPMMap.get(lowerBound).getArmPosition();
        } else if (distance >= upperBound) {
            return trapPositionRPMMap.get(upperBound).getArmPosition();
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

        if (distance <= lowerBound) {
            return trapPositionRPMMap.get(lowerBound).getShooterRPM();
        } else if (distance >= upperBound) {
            return trapPositionRPMMap.get(upperBound).getShooterRPM();
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


    //  public static void main(String[] args) {
    //     System.out.println(PositionRPMData.getInstance().getDesiredShooterRPM(200.0));
    //  }
}
