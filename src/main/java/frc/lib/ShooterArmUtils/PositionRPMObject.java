package frc.lib.ShooterArmUtils;

public class PositionRPMObject {
    private double armPosition;
    private double shooterRPM;

    /**
     * Constructs a new PositionRPMObject.
     * @param armPosition The desired arm position.
     * @param shooterRPM The desired shooter RPM.
     */
    public PositionRPMObject(double armPosition, double shooterRPM) {
        this.armPosition = armPosition;
        this.shooterRPM = shooterRPM;
    }

    /**
     * Method to retrieve the "armPosition" object.
     * @return The position of the arm to be commanded.
     */
    public double getArmPosition() {
        return armPosition;
    }

    /**
     * Method to retrieve the "shooterRPM" object.
     * @return The RPM of the shooter to be commanded.
     */
    public double getShooterRPM() {
        return shooterRPM;
    }

    /**
     * Helper method to set the position value of the arm.
     * @param armPosition The desired arm position to set.
     */
    public void setArmPosition(double armPosition) {
        this.armPosition = armPosition;
    }

    /**
     * Helper method to set the RPM value of the shooter.
     * @param shooterRPM The desired shooter RPM to set.
     */
    public void setShooterRPM(double shooterRPM) {
        this.shooterRPM = shooterRPM;
    }
}