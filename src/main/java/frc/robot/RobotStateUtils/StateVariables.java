package frc.robot.RobotStateUtils;

public class StateVariables {

    public static enum ArmStates {
        HOME(new ArmPosition(-9999)),
        AMP_EJECT(new ArmPosition(-9999)),
        SHOOTING,
        CLIMB(new ArmPosition(-9999));

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

    public static enum ShooterSpeeds {
        STALL(new RPMValue(0)),
        SHOOT,
        AMP_ASSIST(new RPMValue(0));

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

    public static enum IntakeRollerSpeeds {
        OFF(new PercentOutputValue(0)),
        EJECT(new PercentOutputValue(0)),
        INTAKE(new PercentOutputValue(0));

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

    public static enum IntakeStates {
        INTAKE(new IntakePosition(0)),
        HOME(new IntakePosition(0));

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

    public static enum FeederSpeeds {
        GRIP(new PercentOutputValue(0)),
        AMP_EJECT(new PercentOutputValue(0));

        private PercentOutputValue percentOutputValue;

        private FeederSpeeds(PercentOutputValue p) {
            this.percentOutputValue = p;
        }

        public PercentOutputValue percentOutputValue() {
            return percentOutputValue;
        }
    }

}
