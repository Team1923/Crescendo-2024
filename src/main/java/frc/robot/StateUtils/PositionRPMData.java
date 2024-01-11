package frc.robot.StateUtils;

import java.util.HashMap;
import java.util.Set;

public class PositionRPMData {
    private static PositionRPMData positionRPMData;
    private static HashMap<Double, PositionRPMObject> positionRPMMap = new HashMap<>();
    
    public static synchronized PositionRPMData getInstance() {
        if (positionRPMData == null) {
            positionRPMData = new PositionRPMData();
        }

        return positionRPMData;
     }

     private void fillMap() {
        /*
         * IDEA:
         * - Use the limelight to get the distance away from the target.
         * - Store the angle the arm needs to be at and the RPM of the shooter.
         * - Perform a LERP (linear interpolation) to get the value of angle/RPM.
         */
     }

     /**
      * Method to get the desired arm position based on distance.
      * @param distance the limelight's measured distance from target.
      * @return a setpoint for the arm's position.
      */
     public double getDesiredArmPosition(double distance) {
        //TODO: what are the bounds for distance? what to do when distance = 0?
        double lowerBound = 0;
        double upperBound = 0;
        Double[] distanceValues = (Double[]) positionRPMMap.keySet().toArray();

        for (int i = 0; i < distanceValues.length - 1; i++) {
            if (distance >= distanceValues[i] && distance < distanceValues[i + 1]) {
                lowerBound = distanceValues[i];
                upperBound = distanceValues[i + 1];
            }
        }

        double deltaDistance = distance - lowerBound;
        double deltaPositionValues = (positionRPMMap.get(upperBound).getArmPosition()) - 
            (positionRPMMap.get(lowerBound).getArmPosition());
        double deltaDistanceBounds = upperBound - lowerBound;

        return (positionRPMMap.get(lowerBound).getArmPosition()) + ((deltaDistance * deltaPositionValues) / deltaDistanceBounds);
     }

     /**
      * Method to get the desired shooter RPM based on distance.
      * @param distance the limelight's measured distance from the target.
      * @return an RPM setpoint for the shooter's velocity.
      */
     public double getDesiredShooterRPM(double distance) {
        //TODO: what are the bounds for distance? what to do when distance = 0?
        double lowerBound = 0;
        double upperBound = 0;
        Double[] distanceValues = (Double[]) positionRPMMap.keySet().toArray();

        for (int i = 0; i < distanceValues.length - 1; i++) {
            if (distance >= distanceValues[i] && distance < distanceValues[i + 1]) {
                lowerBound = distanceValues[i];
                upperBound = distanceValues[i + 1];
            }
        }

        double deltaDistance = distance - lowerBound;
        double deltaPositionValues = (positionRPMMap.get(upperBound).getShooterRPM()) - 
            (positionRPMMap.get(lowerBound).getShooterRPM());
        double deltaDistanceBounds = upperBound - lowerBound;

        return (positionRPMMap.get(lowerBound).getShooterRPM()) + ((deltaDistance * deltaPositionValues) / deltaDistanceBounds);
     }
}
