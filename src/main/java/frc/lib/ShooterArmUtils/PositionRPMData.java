package frc.lib.ShooterArmUtils;


import java.util.HashMap;

import frc.robot.Constants.LimeLightConstants;

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
        double lowerBound = LimeLightConstants.lerpLowerBound;
        double upperBound = LimeLightConstants.lerpUpperBound;
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
        double lowerBound = LimeLightConstants.lerpLowerBound;
        double upperBound = LimeLightConstants.lerpUpperBound;
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
    //     System.out.println(PositionRPMData.getInstance().getDesiredShooterRPM(84.0));
    //  }
}