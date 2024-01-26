package frc.lib.RobotStateUtils;

public class StateVariables {


    /**
     * TODO: need to change
     */
    public static enum ArmStates {
        STOWED(new ArmPosition(0)),
        AMP(new ArmPosition(1)),
        SPEAKER,
        CLIMB(new ArmPosition(2));

        private ArmPosition armPosition;

        private ArmStates(ArmPosition a) {
            this.armPosition = a;
        }

        private ArmStates() {
            /* Do Nothing, for SHOOTER enum. */
        }

        public ArmPosition getArmPosition() {
            return armPosition;
        }

    }

    public static class ArmPosition {
        private double angularSetpoint;

        public ArmPosition(double a) {
            this.angularSetpoint = a;
        }

        public double getAngularSetpoint() {
            return angularSetpoint;
        }
    }

    /**
     * TODO: These values need to be changed
     */
    public static enum ShooterSpeeds {
        IDLE(new RPMValue(100)),
        RAMP(new RPMValue(200)),
        SHOOT;

        private RPMValue rpmValue;

        private ShooterSpeeds(RPMValue rpmValue) {
            this.rpmValue = rpmValue;
        }

        private ShooterSpeeds() {
            /* Empty constructor for SHOOT enum. */
        }

        public RPMValue getRPMValue() {
            return rpmValue;
        }
    }

    public static class RPMValue {
        private double shooterRPM;

        /**
         * Constucts an instance of RPMValue
         * @param s The value of the shooter RPM you'd like to set.
         */
        public RPMValue(double s) {
            this.shooterRPM = s;
        }

        public double getRPMValue() {
            return shooterRPM;
        }
    }


    /**
     * TODO: These values need to be changed
     */
    public static enum IntakeRollerSpeeds {
        OFF(new PercentOutputValue(0)),
        EJECT(new PercentOutputValue(-100)),
        INTAKE(new PercentOutputValue(100));

        private PercentOutputValue percentOutputValue;

        private IntakeRollerSpeeds(PercentOutputValue p) {
            this.percentOutputValue = p;
        }

        public PercentOutputValue percentOutputValue() {
            return percentOutputValue;
        }
    }

    public static class PercentOutputValue {
        private double percentOutput;

        /**
         * Constucts an instance of PercentOutputValue
         * @param s The intake percent out that you'd like to set.
         */
        public PercentOutputValue(double s) {
            this.percentOutput = s;
        }

        public double getPercentOutputValue() {
            return percentOutput;
        }
    }

    /**
     * TODO: Need to be changed
     */
    public static enum IntakeStates {
        DEPLOYED(new IntakePosition(1)),
        STOWED(new IntakePosition(0));

        private IntakePosition intakePosition;

        /**
         * Creates an IntakeStates object.
         * @param i The IntakePosition object to be passed in.
         */
        private IntakeStates(IntakePosition i) {
            this.intakePosition = i;
        }

        public IntakePosition getIntakePosition() {
            return intakePosition;
        }
    }

    public static class IntakePosition {
        private double angularSetpoint;

        public IntakePosition(double a) {
            this.angularSetpoint = a;
        }

        public double getAngularSetpoint() {
            return angularSetpoint;
        }
    }

    /**
     * TODO: Need to be changed
     */
    public static enum FeederSpeeds {
        OFF(new PercentOutputValue(0)),
        INWARD(new PercentOutputValue(0.5)),
        OUTWARD(new PercentOutputValue(-0.5));

        private PercentOutputValue percentOutputValue;

        private FeederSpeeds(PercentOutputValue p) {
            this.percentOutputValue = p;
        }

        public PercentOutputValue percentOutputValue() {
            return percentOutputValue;
        }
    }

}