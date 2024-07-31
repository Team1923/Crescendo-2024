package frc.robot.lib.StateMachine;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.SwerveRequests.GoalCentricRequest;

public class StateVariables {

    StateHandler stateHandler = StateHandler.getInstance();

    /**
     * TODO: need to change
     */
    public static enum ArmStates {
        STOWED(new ArmPosition(0)),
        REVERSE_SUBWOOFER(new ArmPosition(-1.96)),
        UNGUARDABLE(new ArmPosition(-1.96)),
        AMP(new ArmPosition(-1.96)), 
        SPEAKER(new ArmPosition(-0.77)), //THIS IS A DEFAULT VALUE FOR SUBWOOFER SHOOTING - (-0.77)
        TRAP(new ArmPosition(-0.9)), //TODO: FIND
        BABY_BIRD(new ArmPosition(-0.7)), //TODO: FIND
        PUNT_HIGH(new ArmPosition(-0.72)),
        PUNT_LOW(new ArmPosition(0)),
        FRONT_AMP(new ArmPosition(-0.77)),
        DEFENSE(new ArmPosition(-1.35)),
        CLIMB(new ArmPosition(-1.35));

        private ArmPosition armPosition;

        private ArmStates(ArmPosition a) {
            this.armPosition = a;
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
        IDLE(new RPMValue(0)),
        BABY_BIRD(new RPMValue(-1000)),
        FRONT_AMP_SHOT(new RPMValue(415)),
        UNGUARDABLE_SHOT(new RPMValue(1905)),
        TRAP(new RPMValue(1000)),
        PUNT_SHOT_HIGH(new RPMValue(1)), // this is a percent out
        PUNT_SHOT_LOW(new RPMValue(1)), // this is a percent output value
        SHOOT(new RPMValue(2000));//THIS IS A DEFAULT VALUE FOR A SUBWOOFER SHOT (2000)
        
        private RPMValue rpmValue;

        private ShooterSpeeds(RPMValue rpmValue) {
            this.rpmValue = rpmValue;
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

        public double getRPM() {
            return shooterRPM;
        }
    }


    /**
     * TODO: These values need to be changed
     */
    public static enum IntakeRollerSpeeds {
        OFF(new PercentOutputValue(0)),
        EJECT(new PercentOutputValue(0.75)),
        INTAKE(new PercentOutputValue(-0.85));

        private PercentOutputValue percentOutputValue;

        private IntakeRollerSpeeds(PercentOutputValue p) {
            this.percentOutputValue = p;
        }

        public PercentOutputValue getPercentOutputValue() {
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

        public double getPercentOut() {
            return percentOutput;
        }
    }

    /**
     * TODO: Need to be changed
     */
    public static enum IntakeStates {
        DEPLOYED(new IntakePosition(2.00)),
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
        INWARD(new PercentOutputValue(0.85)),
        OUTWARD(new PercentOutputValue(-0.8)),
        FORWARD(new PercentOutputValue(0.1)),
        BACKING(new PercentOutputValue(-0.1));

        private PercentOutputValue percentOutputValue;

        private FeederSpeeds(PercentOutputValue p) {
            this.percentOutputValue = p;
        }

        public PercentOutputValue getPercentOutputValue() {
            return percentOutputValue;
        }
    }

    public static enum SwerveRequests{
        FIELD_CENTRIC(new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.maxSpeed * 0.1)
            .withRotationalDeadband(SwerveConstants.maxAngularRate * 0.1)
        )
        ,

        ROBOT_CENTRIC(new SwerveRequest.RobotCentric()
            .withDeadband(SwerveConstants.maxSpeed * 0.1)
            .withRotationalDeadband(SwerveConstants.maxAngularRate * 0.1)
        )
        ,
        GOAL_CENTRIC(applyPID(new SwerveRequest.FieldCentricFacingAngle(), new PhoenixPIDController(SwerveConstants.headingKP, SwerveConstants.headingKI, SwerveConstants.headingKD))
            .withDeadband(SwerveConstants.maxSpeed * 0.1)
        )
        ,
        FACING_AMP(applyPID(new SwerveRequest.FieldCentricFacingAngle(), new PhoenixPIDController(SwerveConstants.headingKP, SwerveConstants.headingKI, SwerveConstants.headingKD))
            .withDeadband(SwerveConstants.maxSpeed * 0.1)
        ),
        NOTE_SEARCHING(configedFacingAngleRequest()
        ),
        AUTO(null)
        ;

        public SwerveRequest request;

        private SwerveRequests(SwerveRequest r){
            request = r;

        }

        


    }

    public static SwerveRequest.FieldCentricFacingAngle applyPID(SwerveRequest.FieldCentricFacingAngle r, PhoenixPIDController p){

        r.HeadingController = p;

        return r;
    }

    public static SwerveRequest.FieldCentricFacingAngle configedFacingAngleRequest(){
        SwerveRequest.FieldCentricFacingAngle r = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(SwerveConstants.maxSpeed * 0.1);

        r.HeadingController = new PhoenixPIDController(SwerveConstants.headingKP, SwerveConstants.headingKI, SwerveConstants.headingKD);

        return r;
    }

}
