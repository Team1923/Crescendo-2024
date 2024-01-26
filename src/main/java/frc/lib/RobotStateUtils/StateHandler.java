package frc.lib.RobotStateUtils;

import frc.lib.RobotStateUtils.StateVariables.ArmStates;
import frc.lib.RobotStateUtils.StateVariables.FeederSpeeds;
import frc.lib.RobotStateUtils.StateVariables.IntakeRollerSpeeds;
import frc.lib.RobotStateUtils.StateVariables.IntakeStates;
import frc.lib.RobotStateUtils.StateVariables.ShooterSpeeds;

public class StateHandler {
    private static StateHandler stateHandler;

    public static synchronized StateHandler getInstance() {
        if (stateHandler == null) {
            stateHandler = new StateHandler();
        }

        return stateHandler;
    }

    private ArmStates desiredArmState = ArmStates.STOWED;
    private ShooterSpeeds desiredShootingSpeed = ShooterSpeeds.IDLE;
    private IntakeRollerSpeeds desiredIntakeRollerSpeed = IntakeRollerSpeeds.OFF;
    private IntakeStates desiredIntakeState = IntakeStates.STOWED;
    private FeederSpeeds desiredFeederSpeed = FeederSpeeds.OFF;

    private ArmStates armState = ArmStates.STOWED;
    private ShooterSpeeds shootingSpeed = ShooterSpeeds.IDLE;
    private IntakeRollerSpeeds intakeRollerSpeed = IntakeRollerSpeeds.OFF;
    private IntakeStates intakeState = IntakeStates.STOWED;
    private FeederSpeeds feederSpeed = FeederSpeeds.OFF;

    

    private boolean limelightHasTag = false;
    private double aprilTagID = 0;
    private boolean hasValidSpeakerTag = false;
    private boolean hasValidAmpTag = false;
    private double distanceToTag = 0;

    /**
     * Method to set the desired arm state.
     * @param desiredArmState The desired arm state to be commanded to the arm.
     */
    public void setDesiredArmState(ArmStates desiredArmState) {
        this.desiredArmState = desiredArmState;
    }

    /**
     * Method to get the desired arm state to be commanded.
     * @return The desired arm state.
     */
    public ArmStates getDesiredArmState() {
        return desiredArmState;
    }

    /**
     * Method to get the current arm state to be commanded.
     * @return The current arm state.
     */
    public ArmStates getArmState() {
        return armState;
    }

        /**
     * Method to set the current arm state.
     * @param desiredArmState The current arm state to be commanded to the arm.
     */
    public void setArmState(ArmStates ArmState) {
        this.armState = ArmState;
    }

    /**
     * Sets the desired shooting speed.
     * @param s The desired shooting speed.
     */
    public void setDesiredShootingSpeed(ShooterSpeeds s) {
        this.desiredShootingSpeed = s;
    }

    /**
     * Method to get the desired shooting speed to be commanded.
     * @return The desired shooting speed.
     */
    public ShooterSpeeds getDesiredShootingSpeed() {
        return desiredShootingSpeed;
    }

        /**
     * Sets the current shooting speed.
     * @param s The current shooting speed.
     */
    public void setShootingSpeed(ShooterSpeeds s) {
        this.shootingSpeed = s;
    }

    /**
     * Method to get the current shooting speed to be commanded.
     * @return The current shooting speed.
     */
    public ShooterSpeeds getShootingSpeed() {
        return shootingSpeed;
    }

    /**
     * Sets the desired intake roller speed.
     * @param i The desired intake roller speed.
     */
    public void setDesiredIntakeRollerSpeed(IntakeRollerSpeeds i) {
        this.desiredIntakeRollerSpeed = i;
    }

    /**
     * Method to get the desired intake roller speed to be commanded.
     * @return The desired intake roller speed.
     */
    public IntakeRollerSpeeds getDesiredIntakeRollerSpeed() {
        return desiredIntakeRollerSpeed;
    }

    /**
     * Sets the current intake roller speed.
     * @param i The current intake roller speed.
     */
    public void setIntakeRollerSpeed(IntakeRollerSpeeds i) {
        this.intakeRollerSpeed = i;
    }

    /**
     * Method to get the current intake roller speed to be commanded.
     * @return The current intake roller speed.
     */
    public IntakeRollerSpeeds getIntakeRollerSpeed() {
        return intakeRollerSpeed;
    }

    /**
     * Sets the desired intake state.
     * @param i The desired intake state.
     */
    public void setDesiredIntakeState(IntakeStates i) {
        this.desiredIntakeState = i;
    }

    /**
     * Method to get the desired intake state to be commanded.
     * @return The desired intake state.
     */
    public IntakeStates getDesiredIntakeState() {
        return desiredIntakeState;
    }

    /**
     * Sets the current intake state.
     * @param i The current intake state.
     */
    public void setIntakeState(IntakeStates i) {
        this.intakeState = i;
    }

    /**
     * Method to get the current intake state to be commanded.
     * @return The current intake state.
     */
    public IntakeStates getIntakeState() {
        return intakeState;
    }

    /**
     * Sets the desired feeder speed value.
     * @param f The desired feeder speed.
     */
    public void setDesiredFeederSpeed(FeederSpeeds f) {
        this.desiredFeederSpeed = f;
    }

    /**
     * Method to get the desired feeder speed to be commanded.
     * @return The desired feeder speed.
     */
    public FeederSpeeds getDesiredFeederSpeed() {
        return desiredFeederSpeed;
    }

    /**
     * Sets the current feeder speed value.
     * @param f The current feeder speed.
     */
    public void setFeederSpeed(FeederSpeeds f) {
        this.feederSpeed = f;
    }

    /**
     * Method to get the current feeder speed to be commanded.
     * @return The current feeder speed.
     */
    public FeederSpeeds getFeederSpeed() {
        return feederSpeed;
    }

   
    /**
     * Sets if the Limelight has a valid target.
     * @param limelightHasTarget The boolean value to determine if the Limelight has a valid target.
     */
    public void setLimelightHasTag(boolean limelightHasTag) {
        this.limelightHasTag = limelightHasTag;
    }

    /**
     * Gets if the Limelight has a valid target.
     * @return The boolean value if Limelight has a valid target.
     */
     public boolean getLimelightHasTag() {
        return limelightHasTag;
    }

    /**
     * Sets the ID of the April Tag in frame.
     * @param aprilTagID The double value that sets the Tag ID.
     */
    public void setAprilTagID(double aprilTagID) {
        this.aprilTagID = aprilTagID;
    }

    /**
     * Gets the ID of the April Tag in Frame;
     * @return The double value of the TAG ID
     */
    public double getAprilTagID() {
        return aprilTagID;
    }

    /**
     * Sets the boolean of whether or not there is a valid Speaker April Tag.
     * @param hasValidSpeakerTag The boolean which determines if there is a valid Speaker April Tag.
     */
    public void setHasValidSpeakerTag(boolean hasValidSpeakerTag) {
        this.hasValidSpeakerTag = hasValidSpeakerTag;
    }

    /**
     * Gets the boolean of whether or not there is a valid Speaker April Tag.
     * @return The boolean of whether or not there is a valid Speaker April Tag.
     */
    public boolean getHasValidSpeakerTag() {
        return hasValidSpeakerTag;
    }

    /**
     * Sets the boolean of whether or not there is a valid Amp April Tag.
     * @param hasValidAmpTag The boolean which determines if there is a valid Amp April Tag.
     */
    public void setHasValidAmpTag(boolean hasValidAmpTag) {
        this.hasValidAmpTag = hasValidAmpTag;
    }

    /**
     * Gets the boolean of whether or not there is a valid Amp April Tag.
     * @return The boolean of whether or not there is a valid Amp April Tag.
     */
    public boolean getHasValidAmpTag() {
        return hasValidAmpTag;
    }

    /**
     * Sets the distance to an April Tag.
     * @param distanceToTag The double which is passed in to set the distance to the April Tag.
     */
    public void setDistanceToTag(double distanceToTag) {
        this.distanceToTag = distanceToTag;
    }

    /**
     * Gets the distance to an April Tag.
     * @return A double which is the distance to the April Tag.
     */
    public double getDistanceToTag() {
        return distanceToTag;
    }

}