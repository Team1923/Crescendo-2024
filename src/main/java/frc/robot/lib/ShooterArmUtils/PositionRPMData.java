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
        speakerPositionRPMMap.put(149.25, new PositionRPMObject(-0.255, 4000));
        speakerPositionRPMMap.put(153.1, new PositionRPMObject(-0.255, 4000));
        speakerPositionRPMMap.put(103.6, new PositionRPMObject(-0.4, 3000));
        speakerPositionRPMMap.put(137.0, new PositionRPMObject(-0.29, 4000));
        speakerPositionRPMMap.put(222.7, new PositionRPMObject(0, 4200));
        speakerPositionRPMMap.put(105.1, new PositionRPMObject(-0.36, 3000));
        speakerPositionRPMMap.put(235.9, new PositionRPMObject(0, 4500));
        speakerPositionRPMMap.put(231.6, new PositionRPMObject(0, 4500));
        speakerPositionRPMMap.put(198.2, new PositionRPMObject(-0.165, 4200));
        speakerPositionRPMMap.put(181.5, new PositionRPMObject(-0.185, 4200));
        speakerPositionRPMMap.put(159.4, new PositionRPMObject(-0.205, 4000));
        speakerPositionRPMMap.put(92.1, new PositionRPMObject(-0.45, 3000));
        speakerPositionRPMMap.put(78.7, new PositionRPMObject(-0.5, 3000));
        speakerPositionRPMMap.put(56.6, new PositionRPMObject(-0.6, 3000));
        speakerPositionRPMMap.put(52.3, new PositionRPMObject(-0.65, 3000));


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
