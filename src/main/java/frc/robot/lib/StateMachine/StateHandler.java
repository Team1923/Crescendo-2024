package frc.robot.lib.StateMachine;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;
import frc.robot.lib.StateMachine.StateVariables.FeederSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeRollerSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeStates;
import frc.robot.lib.StateMachine.StateVariables.ShooterSpeeds;

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

    /* Limelight */
    private boolean limelightHasTag = false;
    private int aprilTagID = 0;
    private boolean hasValidSpeakerTag = false;
    private boolean hasValidTrapTag = false;
    private boolean hasValidAmpTag = false;
    private double distanceToSpeakerTag = 0;
    private double distanceToTrapTag = 0;
    private boolean centeredToTag = false;
    private boolean wantGoalCentric = false;
    private double xAngleOffset = 0;

    public double getxAngleOffset() {
        return xAngleOffset;
    }

    public void setxAngleOffset(double xAngleOffset) {
        this.xAngleOffset = xAngleOffset;
    }

    /* Desired scoring */
    private boolean wantToPositionForSubwoofer = false;
    private boolean scoreInAmp = false;
    private boolean scoreInTrap = false;
    private boolean wantToPositionForReverseSubwoofer = false;
    private boolean wantToPunt = false;
    private boolean wantUnguardable = false;

    /* PosRPM tuning values */
    private double positionOffset = 0;
    private double RPMOffset = 0;
    private boolean posRPMTuning = false;

    /* BEAM BREAK Values */
    private boolean bbOneCovered = false;
    private boolean bbTwoCovered = false;
    private boolean bbThreeCovered = false;
    private boolean bbFourCovered = false;

    /* Misc */
    private boolean manuallyClimbing = false;
    private boolean fullEject = false;
    private boolean operatorInputTimingGood = false;
    private boolean autoOverride = false;
    private boolean pooPooPeePeeBB1Died = false;
    private Pose2d robotPose = new Pose2d();
    private Pose3d currentTagPose = new Pose3d();

    
    public Pose3d getCurrentTagPose() {
        return currentTagPose;
    }

    public void setCurrentTagPose(Pose3d currentTagPose) {
        this.currentTagPose = currentTagPose;
    }

    private double coveredSpeakerTagDistance = 0;
    

    public double getCoveredSpeakerTagDistance() {
        return coveredSpeakerTagDistance;
    }

    public void setCoveredSpeakerTagDistance(double coveredSpeakerTagDistance) {
        this.coveredSpeakerTagDistance = coveredSpeakerTagDistance;
    }

    /**
     * Method to set the desired arm state.
     * 
     * @param desiredArmState The desired arm state to be commanded to the arm.
     */
    public void setDesiredArmState(ArmStates desiredArmState) {
        this.desiredArmState = desiredArmState;
    }

    /**
     * Method to get the desired arm state to be commanded.
     * 
     * @return The desired arm state.
     */
    public ArmStates getDesiredArmState() {
        return desiredArmState;
    }

    /**
     * Method to get the current arm state to be commanded.
     * 
     * @return The current arm state.
     */
    public ArmStates getCurrentArmState() {
        return currentArmState;
    }

    /**
     * Method to set the current arm state.
     * 
     * @param desiredArmState The current arm state to be commanded to the arm.
     */
    public void setCurrentArmState(ArmStates ArmState) {
        this.currentArmState = ArmState;
    }

    /**
     * Sets the desired shooting speed.
     * 
     * @param s The desired shooting speed.
     */
    public void setDesiredShootingSpeed(ShooterSpeeds s) {
        this.desiredShootingSpeed = s;
    }

    /**
     * Method to get the desired shooting speed to be commanded.
     * 
     * @return The desired shooting speed.
     */
    public ShooterSpeeds getDesiredShootingSpeed() {
        return desiredShootingSpeed;
    }

    /**
     * Sets the current shooting speed.
     * 
     * @param s The current shooting speed.
     */
    public void setCurrentShootingSpeed(ShooterSpeeds s) {
        this.currentShootingSpeed = s;
    }

    /**
     * Method to get the current shooting speed to be commanded.
     * 
     * @return The current shooting speed.
     */
    public ShooterSpeeds getCurrentShootingSpeed() {
        return currentShootingSpeed;
    }

    /**
     * Sets the desired intake roller speed.
     * 
     * @param i The desired intake roller speed.
     */
    public void setDesiredIntakeRollerSpeed(IntakeRollerSpeeds i) {
        this.desiredIntakeRollerSpeed = i;
    }

    /**
     * Method to get the desired intake roller speed to be commanded.
     * 
     * @return The desired intake roller speed.
     */
    public IntakeRollerSpeeds getDesiredIntakeRollerSpeed() {
        return desiredIntakeRollerSpeed;
    }

    /**
     * Sets the current intake roller speed.
     * 
     * @param i The current intake roller speed.
     */
    public void setCurrentIntakeRollerSpeed(IntakeRollerSpeeds i) {
        this.currentIntakeRollerSpeed = i;
    }

    /**
     * Method to get the current intake roller speed to be commanded.
     * 
     * @return The current intake roller speed.
     */
    public IntakeRollerSpeeds getCurrentIntakeRollerSpeed() {
        return currentIntakeRollerSpeed;
    }

    /**
     * Sets the desired intake state.
     * 
     * @param i The desired intake state.
     */
    public void setDesiredIntakeState(IntakeStates i) {
        this.desiredIntakeState = i;
    }

    /**
     * Method to get the desired intake state to be commanded.
     * 
     * @return The desired intake state.
     */
    public IntakeStates getDesiredIntakeState() {
        return desiredIntakeState;
    }

    /**
     * Sets the current intake state.
     * 
     * @param i The current intake state.
     */
    public void setCurrentIntakeState(IntakeStates i) {
        this.currentIntakeState = i;
    }

    /**
     * Method to get the current intake state to be commanded.
     * 
     * @return The current intake state.
     */
    public IntakeStates getCurrentIntakeState() {
        return currentIntakeState;
    }

    /**
     * Sets the desired feeder speed value.
     * 
     * @param f The desired feeder speed.
     */
    public void setDesiredFeederSpeed(FeederSpeeds f) {
        this.desiredFeederSpeed = f;
    }

    /**
     * Method to get the desired feeder speed to be commanded.
     * 
     * @return The desired feeder speed.
     */
    public FeederSpeeds getDesiredFeederSpeed() {
        return desiredFeederSpeed;
    }

    /**
     * Sets the current feeder speed value.
     * 
     * @param f The current feeder speed.
     */
    public void setCurrentFeederSpeed(FeederSpeeds f) {
        this.currentFeederSpeed = f;
    }

    /**
     * Method to get the current feeder speed to be commanded.
     * 
     * @return The current feeder speed.
     */
    public FeederSpeeds getCurrentFeederSpeed() {
        return currentFeederSpeed;
    }

    /**
     * Sets if the Limelight has a valid target.
     * 
     * @param limelightHasTarget The boolean value to determine if the Limelight has
     *                           a valid target.
     */
    public void setLimelightHasTag(boolean limelightHasTag) {
        this.limelightHasTag = limelightHasTag;
    }

    /**
     * Gets if the Limelight has a valid target.
     * 
     * @return The boolean value if Limelight has a valid target.
     */
    public boolean getLimelightHasTag() {
        return limelightHasTag;
    }

    /**
     * Sets the ID of the April Tag in frame.
     * 
     * @param aprilTagID The double value that sets the Tag ID.
     */
    public void setAprilTagID(int aprilTagID) {
        this.aprilTagID = aprilTagID;
    }

    /**
     * Gets the ID of the April Tag in Frame;
     * 
     * @return The double value of the TAG ID
     */
    public int getAprilTagID() {
        return aprilTagID;
    }

    /**
     * Sets the boolean of whether or not there is a valid Trap April Tag.
     * 
     * @param hasValidTrapTag The boolean which determines if there is a valid
     *                           Trap April Tag.
     */
    public void setHasValidTrapTag(boolean hasValidTrapTag) {
        this.hasValidTrapTag = hasValidTrapTag;
    }

    /**
     * Gets the boolean of whether or not there is a valid Trap April Tag.
     * 
     * @return The boolean of whether or not there is a valid Trap April Tag.
     */
    public boolean getHasValidTrapTag() {
        return hasValidTrapTag;
    }

    /**
     * Sets the boolean of whether or not there is a valid Speaker April Tag.
     * 
     * @param hasValidSpeakerTag The boolean which determines if there is a valid
     *                           Speaker April Tag.
     */
    public void setHasValidSpeakerTag(boolean hasValidSpeakerTag) {
        this.hasValidSpeakerTag = hasValidSpeakerTag;
    }

    /**
     * Gets the boolean of whether or not there is a valid Speaker April Tag.
     * 
     * @return The boolean of whether or not there is a valid Speaker April Tag.
     */
    public boolean getHasValidSpeakerTag() {
        return hasValidSpeakerTag;
    }

    /**
     * Sets the boolean of whether or not there is a valid Amp April Tag.
     * 
     * @param hasValidAmpTag The boolean which determines if there is a valid Amp
     *                       April Tag.
     */
    public void setHasValidAmpTag(boolean hasValidAmpTag) {
        this.hasValidAmpTag = hasValidAmpTag;
    }

    /**
     * Gets the boolean of whether or not there is a valid Amp April Tag.
     * 
     * @return The boolean of whether or not there is a valid Amp April Tag.
     */
    public boolean getHasValidAmpTag() {
        return hasValidAmpTag;
    }

    /**
     * Sets the distance to speaker April Tag.
     * 
     * @param distanceToTag The double which is passed in to set the distance to the
     *                      April Tag.
     */
    public void setDistanceToSpeakerTag(double distanceToTag) {
        this.distanceToSpeakerTag = distanceToTag;
    }

    /**
     * Gets the distance to speaker April Tag.
     * 
     * @return A double which is the distance to the April Tag.
     */
    public double getDistanceToSpeakerTag() {
        return distanceToSpeakerTag;
    }

    /**
     * Sets the distance to trap April Tag.
     * 
     * @param distanceToTag The double which is passed in to set the distance to the
     *                      April Tag.
     */
    public void setDistanceToTrapTag(double distanceToTag) {
        this.distanceToTrapTag = distanceToTag;
    }

    /**
     * Gets the distance to trap April Tag.
     * 
     * @return A double which is the distance to the April Tag.
     */
    public double getDistanceToTrapTag() {
        return distanceToTrapTag;
    }

    /**
     * Method to set the current state of Beam Break One.
     * Use the current reading from Beam Break One to set
     * a more accessible reading within StateHandler.
     * 
     * @param isCovered boolean variable to determine if BB One is covered.
     */
    public void setBBOneCovered(boolean isCovered) {
        this.bbOneCovered = isCovered;
    }

    /**
     * Method to get the current state of Beam Break One.
     * 
     * @return a boolean variable to determine if BB One is covered.
     */
    public boolean getBBOneCovered() {
        return bbOneCovered;
    }

    /**
     * Method to set the current state of Beam Break Two.
     * Use the current reading from Beam Break Two to set
     * a more accessible reading within StateHandler.
     * 
     * @param isCovered boolean variable to determine if BB Two is covered.
     */
    public void setBBTwoCovered(boolean isCovered) {
        this.bbTwoCovered = isCovered;
    }

    /**
     * Method to get the current state of Beam Break Two.
     * 
     * @return a boolean variable to determine if BB Two is covered.
     */
    public boolean getBBTwoCovered() {
        return bbTwoCovered;
    }

    /**
     * Method to set the current state of Beam Break Three.
     * Use the current reading from Beam Break Three to set
     * a more accessible reading within StateHandler.
     * 
     * @param isCovered boolean variable to determine if BB Three is covered.
     */
    public void setBBThreeCovered(boolean isCovered) {
        this.bbThreeCovered = isCovered;
    }

    /**
     * Method to get the current state of Beam Break Three.
     * 
     * @return a boolean variable to determine if BB Three is covered.
     */
    public boolean getBBThreeCovered() {
        return bbThreeCovered;
    }

    /**
     * Method to set the current state of Beam Break Four.
     * Use the current reading from Beam Break Four to set
     * a more accessible reading within StateHandler.
     * 
     * @param isCovered boolean variable to determine if BB Four is covered.
     */
    public void setBBFourCovered(boolean isCovered) {
        this.bbFourCovered = isCovered;
    }

    /**
     * Method to get the current state of Beam Break Four.
     * 
     * @return a boolean variable to determine if BB Four is covered.
     */
    public boolean getBBFourCovered() {
        return bbFourCovered;
    }

    /**
     * Sets a boolean that determines if the swerve drive is centered onto the AprilTag.
     * @param centeredToTag the boolean to set to determine if the drivebase is centered onto the AprilTag.
     */
    public void setIsCenteredToTag(boolean centeredToTag) {
        this.centeredToTag = centeredToTag;
    }

    /**
     * Returns if the swerve drive is centered onto the AprilTag.
     * @return a boolean that represents if the swerve drive is centered onto the april tag.
     */
    public boolean getIsCenteredToTag() {
        return centeredToTag;
    }

    /**
     * Sets the boolean to determine if the arm should be positioned for a subwoofer shot.
     * @param s boolean value to set to determine if the arm should be positioned for a subwoofer shot.
     */
    public void setScoreInSubwoofer(boolean s) {
        this.wantToPositionForSubwoofer = s;
    }

    /**
     * Returns if the arm should be positioned for a subwoofer shot.
     * @return a boolean that represents if the arm should be positioned for the subwoofer.
     */
    public boolean getScoreInSubwoofer() {
        return wantToPositionForSubwoofer;
    }

    /**
     * Sets the boolean to determine if the arm should be positioned for an amp score.
     * @param a boolean value to set to determine if the arm should be positioned for an amp score.
     */
    public void setScoreInAmp(boolean a) {
        this.scoreInAmp = a;
    }

    /**
     * Returns if the arm should be positioned for an amp score.
     * @return a boolean that reperesents if the arm should be positioned for the amp.
     */
    public boolean getScoreInAmp() {
        return scoreInAmp;
    }

    public void setScoreInTrap(boolean a) {
        this.scoreInTrap = a;
    }

    public boolean getScoreInTrap() {
        return scoreInTrap;
    }

    /**
     * Sets the boolean to determine if we want to score in the Subwoofer with the Reversed Subwoofer state.
     * @param a a boolean value to set to determine if the arm should be positioned reversed subwoofer.
     */
    public void setScoreInReverseSubwoofer(boolean a){
        this.wantToPositionForReverseSubwoofer = a;
    }

    /**
     * Returns if the arm should be positioned for reverseSubwoofer.
     * @return a boolean that represents if the arm should be positioned for reverse subwoofer.
     */
    public boolean getScoreInReverseSubwoofer(){
        return wantToPositionForReverseSubwoofer;
    }

   /**
    * Returns if the arm is ready for manual climb.
    * @return A boolean value that represents if the arm is ready for manual climb.
    */
    public boolean getManuallyClimbing() {
        return manuallyClimbing;
    }

    /**
     * Sets the boolean to determine if we want to move the arm to climb, and are ready for manual climb.
     * @param a a boolean value to set to determine if the arm is ready for manual climb.
     */
    public void setManuallyClimbing(boolean manuallyClimbing) {
        this.manuallyClimbing = manuallyClimbing;
    }

    /**
     * Method to retrieve whether or not the arm/shooter is in PUNT
     * @return a boolean representation of the above.
     */
    public boolean getWantPunt() {
        return wantToPunt;
    }

    /**
     * Method to set the desired shooting/arm state to PUNT
     * @param p whether or not punting is desired.
     */
    public void setWantPunt(boolean p) {
        this.wantToPunt = p;
    }

    public boolean getOperatorInputTimingGood() {
        return operatorInputTimingGood;
    }

    public void setOperatorInputTimingGood(boolean operatorInputTimingGood) {
        this.operatorInputTimingGood = operatorInputTimingGood;
    }

    public boolean getWantUnguardable() {
        return wantUnguardable;
    }

    public void setWantUnguardable(boolean wantUnguardable) {
        this.wantUnguardable = wantUnguardable;
    }


    public boolean getIsGoalCentric() {
        return wantGoalCentric;
    }

    public void setIsGoalCentric(boolean wantGoalCentric) {
        this.wantGoalCentric = wantGoalCentric;
    }

    public boolean getFullEject() {
        return fullEject;
    }

    public void setFullEject(boolean fullEject) {
        this.fullEject = fullEject;
    }

     public boolean getAutoOverride() {
        return autoOverride;
    }

    public void setAutoOverride(boolean autoOverride) {
        this.autoOverride = autoOverride;
    }

    public boolean getBB1Dead() {
        return pooPooPeePeeBB1Died;
    }

    public void setBB1Dead(boolean noBB1) {
        this.pooPooPeePeeBB1Died = noBB1;
    }


    //HASHMAP TUNING


    public double getPositionOffset() {
        return positionOffset;
    }

    public void setPositionOffset(double positionOffset) {
        this.positionOffset = positionOffset;
    }

    public double getRPMOffset() {
        return RPMOffset;
    }

    public void setRPMOffset(double rPMOffset) {
        RPMOffset = rPMOffset;
    }

     public boolean isPosRPMTuning() {
        return posRPMTuning;
    }


    public Pose2d getRobotPose() {
        return robotPose;
    }

    public void setRobotPose(Pose2d robotPose) {
        this.robotPose = robotPose;
    }
    

}