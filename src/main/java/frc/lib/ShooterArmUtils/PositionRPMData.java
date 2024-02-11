package frc.lib.ShooterArmUtils;


import java.util.HashMap;

public class PositionRPMData {
    private static PositionRPMData positionRPMData;
    private static HashMap<Double, PositionRPMObject> positionRPMMap = new HashMap<>();
    
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
        fillMap();
     }

     private void fillMap() {
        /*
         * IDEA:
         * - Use the limelight to get the distance away from the target.
         * - Store the angle the arm needs to be at and the RPM of the shooter.
         * - Perform a LERP (linear interpolation) to get the value of angle/RPM.
         * 
         * For now I'm trying test data to see if the linear interpolation works.
         */
        // positionRPMMap.put(12 * (0.725 + 3.33333), new PositionRPMObject(-0.751, 5000));
        // positionRPMMap.put(12 * (1.45 + 3.33333), new PositionRPMObject(-0.711, 5000));
        // positionRPMMap.put(12 * (2.175 + 3.33333), new PositionRPMObject(-0.671, 5000));
        // positionRPMMap.put(12 * (2.9 + 3.33333), new PositionRPMObject(-0.632, 5000));
        // positionRPMMap.put(12 * (3.625 + 3.33333), new PositionRPMObject(-0.592, 5000));
        // positionRPMMap.put(12 * (4.35 + 3.33333), new PositionRPMObject(-0.553, 5000));
        // positionRPMMap.put(12 * (5.075 + 3.33333), new PositionRPMObject(-0.513, 5000));
        // positionRPMMap.put(12 * (5.8 + 3.33333), new PositionRPMObject(-0.474, 5000));
        // positionRPMMap.put(12 * (6.525 + 3.33333), new PositionRPMObject(-0.434, 5000));
        // positionRPMMap.put(12 * (7.25 + 3.33333), new PositionRPMObject(-0.395, 5000));
        // positionRPMMap.put(12 * (7.975 + 3.33333), new PositionRPMObject(-0.355, 5000));
        // positionRPMMap.put(12 * (8.7 + 3.33333), new PositionRPMObject(-0.316, 5000));
        // positionRPMMap.put(12 * (9.425 + 3.33333), new PositionRPMObject(-0.276, 5000));
        // positionRPMMap.put(12 * (10.15 + 3.33333), new PositionRPMObject(-0.237, 5000));
        // positionRPMMap.put(12 * (10.875 + 3.33333), new PositionRPMObject(-0.197, 5000));
        // positionRPMMap.put(12 * (11.6 + 3.33333), new PositionRPMObject(-0.158, 5000));
        // positionRPMMap.put(12 * (12.325 + 3.33333), new PositionRPMObject(-0.118, 5000));
        // positionRPMMap.put(12 * (13.05 + 3.33333), new PositionRPMObject(-0.079, 5000));
        // positionRPMMap.put(160.0, new PositionRPMObject(-0.039, 5000));
        // positionRPMMap.put(166.0, new PositionRPMObject(0, 5000));
        // positionRPMMap.put(190.0, new PositionRPMObject(0, 5000));
        // positionRPMMap.put(-200.0, new PositionRPMObject(-0.52, 3000));
        // positionRPMMap.put(1000.0, new PositionRPMObject(-0.52, 3000));
        positionRPMMap.put(168.0, new PositionRPMObject(0, 5000));
        positionRPMMap.put(156.0, new PositionRPMObject(-0.212, 4000));
        positionRPMMap.put(144.0, new PositionRPMObject(-0.235, 4000));
        positionRPMMap.put(132.0, new PositionRPMObject(-0.258, 4000));
        positionRPMMap.put(120.0, new PositionRPMObject(-0.281, 4000));
        positionRPMMap.put(108.0, new PositionRPMObject(-0.304, 4000));
        positionRPMMap.put(84.0, new PositionRPMObject(-0.52, 3000));




    
        

     }

     /**
      * Method to get the desired arm position based on distance.
      * @param distance the limelight's measured distance from target.
      * @return a setpoint for the arm's position.
      */
     public double getDesiredArmPosition(double distance) {
        //TODO: what are the bounds for distance? what to do when distance = 0? Handle edge cases.
        double lowerBound = 0;
        double upperBound = 0;
        Double[] distanceValues = (Double[]) positionRPMMap.keySet().toArray(new Double[positionRPMMap.size()]);

        for (int i = 0; i < distanceValues.length - 1; i++) {
            if (distance >= distanceValues[i] && distance <= distanceValues[i + 1]) {
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
        //TODO: what are the bounds for distance? what to do when distance = 0? Handle edge cases.
        double lowerBound = 0;
        double upperBound = 0;
        Double[] distanceValues = (Double[]) positionRPMMap.keySet().toArray(new Double[positionRPMMap.size()]);

        for (int i = 0; i < distanceValues.length - 1; i++) {
            if (distance >= distanceValues[i] && distance <= distanceValues[i + 1]) {
                lowerBound = distanceValues[i];
                upperBound = distanceValues[i + 1];
            }
        }

        double deltaDistance = distance - lowerBound;
        double deltaRPMValues = (positionRPMMap.get(upperBound).getShooterRPM()) - 
            (positionRPMMap.get(lowerBound).getShooterRPM());
        double deltaDistanceBounds = upperBound - lowerBound;

        return (positionRPMMap.get(lowerBound).getShooterRPM()) + ((deltaDistance * deltaRPMValues) / deltaDistanceBounds);
     }

     public static void main(String[] args) {
        double slope = 0.05448275862069;
        double yInt = -0.79;
        double stepInterval = 0.725;
        for (int i = 1; i <= 20; i++) {
            double distance = stepInterval * i;
            System.out.println("DISTANCE: " + distance + " ANGLE: " + (slope * distance + yInt));
        } 
     }
}