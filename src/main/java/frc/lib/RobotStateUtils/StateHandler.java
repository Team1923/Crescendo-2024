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

    /* DESIRED STATES: These tell mechanisms to "go" to the state specified. */
    private ArmStates desiredArmState = ArmStates.STOWED;
    private ShooterSpeeds desiredShootingSpeed = ShooterSpeeds.IDLE;
    private IntakeRollerSpeeds desiredIntakeRollerSpeed = IntakeRollerSpeeds.OFF;
    private IntakeStates desiredIntakeState = IntakeStates.STOWED;
    private FeederSpeeds desiredFeederSpeed = FeederSpeeds.OFF;

    /* CURRENT STATES: Mechanisms have arrived to the desired state. */
    private ArmStates currentArmState = ArmStates.STOWED;
    private ShooterSpeeds currentShootingSpeed = ShooterSpeeds.IDLE;
    private IntakeRollerSpeeds currentIntakeRollerSpeed = IntakeRollerSpeeds.OFF;
    private IntakeStates currentIntakeState = IntakeStates.STOWED;
    private FeederSpeeds currentFeederSpeed = FeederSpeeds.OFF;

    /* Extra variables needed to handle additional states. */
    private boolean limelightHasTag = false;
    private double aprilTagID = 0;
    private boolean hasValidSpeakerTag = false;
    private boolean hasValidAmpTag = false;
    private double distanceToTag = 0;

    /* BEAM BREAK Values */
    private boolean bbOneCovered = false;
    private boolean bbTwoCovered = false;
    private boolean bbThreeCovered = false;
    private boolean bbFourCovered = false;


    private boolean scoreInAmp = false;
    

    private boolean scoreInSubwoofer = false;
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
    public ArmStates getCurrentArmState() {
        return currentArmState;
    }

    /**
     * Method to set the current arm state.
     * @param desiredArmState The current arm state to be commanded to the arm.
     */
    public void setCurrentArmState(ArmStates ArmState) {
        this.currentArmState = ArmState;
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
    public void setCurrentShootingSpeed(ShooterSpeeds s) {
        this.currentShootingSpeed = s;
    }

    /**
     * Method to get the current shooting speed to be commanded.
     * @return The current shooting speed.
     */
    public ShooterSpeeds getCurrentShootingSpeed() {
        return currentShootingSpeed;
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
    public void setCurrentIntakeRollerSpeed(IntakeRollerSpeeds i) {
        this.currentIntakeRollerSpeed = i;
    }

    /**
     * Method to get the current intake roller speed to be commanded.
     * @return The current intake roller speed.
     */
    public IntakeRollerSpeeds getCurrentIntakeRollerSpeed() {
        return currentIntakeRollerSpeed;
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
    public void setCurrentIntakeState(IntakeStates i) {
        this.currentIntakeState = i;
    }

    /**
     * Method to get the current intake state to be commanded.
     * @return The current intake state.
     */
    public IntakeStates getCurrentIntakeState() {
        return currentIntakeState;
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
    public void setCurrentFeederSpeed(FeederSpeeds f) {
        this.currentFeederSpeed = f;
    }

    /**
     * Method to get the current feeder speed to be commanded.
     * @return The current feeder speed.
     */
    public FeederSpeeds getCurrentFeederSpeed() {
        return currentFeederSpeed;
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

    /**
     * Method to set the current state of Beam Break One.
     * Use the current reading from Beam Break One to set 
     * a more accessible reading within StateHandler.
     * @param isCovered boolean variable to determine if BB One is covered.
     */
    public void setBBOneCovered(boolean isCovered) {
        this.bbOneCovered = isCovered;
    }

    /**
     * Method to get the current state of Beam Break One.
     * @return a boolean variable to determine if BB One is covered.
     */
    public boolean getBBOneCovered() {
        return bbOneCovered;
    }


    /**
     * Method to set the current state of Beam Break Two.
     * Use the current reading from Beam Break Two to set 
     * a more accessible reading within StateHandler.
     * @param isCovered boolean variable to determine if BB Two is covered.
     */
    public void setBBTwoCovered(boolean isCovered) {
        this.bbTwoCovered = isCovered;
    }

    /**
     * Method to get the current state of Beam Break Two.
     * @return a boolean variable to determine if BB Two is covered.
     */
    public boolean getBBTwoCovered() {
        return bbTwoCovered;
    }

    /**
     * Method to set the current state of Beam Break Three.
     * Use the current reading from Beam Break Three to set 
     * a more accessible reading within StateHandler.
     * @param isCovered boolean variable to determine if BB Three is covered.
     */
    public void setBBThreeCovered(boolean isCovered) {
        this.bbThreeCovered = isCovered;
    }

    /**
     * Method to get the current state of Beam Break Three.
     * @return a boolean variable to determine if BB Three is covered.
     */
    public boolean getBBThreeCovered() {
        return bbThreeCovered;
    }

    /**
     * Method to set the current state of Beam Break Four.
     * Use the current reading from Beam Break Four to set 
     * a more accessible reading within StateHandler.
     * @param isCovered boolean variable to determine if BB Four is covered.
     */
    public void setBBFourCovered(boolean isCovered) {
        this.bbFourCovered = isCovered;
    }

    /**
     * Method to get the current state of Beam Break Four.
     * @return a boolean variable to determine if BB Four is covered.
     */
    public boolean getBBFourCovered() {
        return bbFourCovered;
    }




    public boolean getScoreInAmp() {
        return scoreInAmp;
    }

    public void setScoreInAmp(boolean scoreInAmp) {
        this.scoreInAmp = scoreInAmp;
    }

    public boolean getScoreInSubwoofer() {
        return scoreInSubwoofer;
    }

    public void setScoreInSubwoofer(boolean scoreInSubwoofer) {
        this.scoreInSubwoofer = scoreInSubwoofer;
    }

}