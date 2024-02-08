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

        positionRPMMap.put(1.0, new PositionRPMObject(5, 2000));
        positionRPMMap.put(1.5, new PositionRPMObject(7, 2250));
        positionRPMMap.put(2.0, new PositionRPMObject(9, 2500));
        positionRPMMap.put(2.5, new PositionRPMObject(14, 2600));
        positionRPMMap.put(3.0, new PositionRPMObject(17, 2800));
        positionRPMMap.put(3.5, new PositionRPMObject(19, 3000));
        positionRPMMap.put(4.0, new PositionRPMObject(23, 3100));
        positionRPMMap.put(4.5, new PositionRPMObject(14, 3400));

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

    //  public static void main(String[] args) {
    //     PositionRPMData test = PositionRPMData.getInstance();
    //     System.out.println("DESIRED ARM POSITION: " + test.getDesiredArmPosition(1.0));
    //     System.out.println("DESIRED RPM: " + test.getDesiredShooterRPM(1.0));    
    //  }
}