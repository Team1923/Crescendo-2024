package frc.robot.RobotStateUtils;

import frc.robot.RobotStateUtils.StateVariables.ArmStates;
import frc.robot.RobotStateUtils.StateVariables.FeederSpeeds;
import frc.robot.RobotStateUtils.StateVariables.IntakeRollerSpeeds;
import frc.robot.RobotStateUtils.StateVariables.IntakeStates;
import frc.robot.RobotStateUtils.StateVariables.ShooterSpeeds;

public class StateHandler {
    private static StateHandler stateHandler;

    public static synchronized StateHandler getInstance() {
        if (stateHandler == null) {
            stateHandler = new StateHandler();
        }

        return stateHandler;
    }

    private ArmStates desiredArmState = ArmStates.HOME;
    private ShooterSpeeds desiredShootingSpeed = ShooterSpeeds.STALL;
    private IntakeRollerSpeeds desiredIntakeRollerSpeed = IntakeRollerSpeeds.OFF;
    private IntakeStates desiredIntakeState = IntakeStates.HOME;
    private FeederSpeeds desiredFeederSpeed = FeederSpeeds.GRIP;

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

}
