package frc.robot.lib.SwerveRequests;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GoalCentricRequest extends SwerveRequest.FieldCentricFacingAngle{



        public DoubleSupplier HeadingSupplier = () -> 0;

        public DoubleSupplier TxSupplier = () -> 0;

        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            double toApplyX = VelocityX;
            double toApplyY = VelocityY;
            
            Rotation2d angleToFace = Rotation2d.fromDegrees(HeadingSupplier.getAsDouble()+TxSupplier.getAsDouble());
            if (ForwardReference == SwerveRequest.ForwardReference.OperatorPerspective) {
                /* If we're operator perspective, modify the X/Y translation by the angle */
                Translation2d tmp = new Translation2d(toApplyX, toApplyY);
                tmp = tmp.rotateBy(parameters.operatorForwardDirection);
                toApplyX = tmp.getX();
                toApplyY = tmp.getY();
                /* And rotate the direction we want to face by the angle */
                angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
            }

            double rotationRate = HeadingController.calculate(parameters.currentPose.getRotation().getRadians(),
                    angleToFace.getRadians(), parameters.timestamp);

            double toApplyOmega = rotationRate;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }

            ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                    parameters.currentPose.getRotation()), parameters.updatePeriod);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }


        public GoalCentricRequest withHeadingSupplier(DoubleSupplier h){
            this.HeadingSupplier = h;
            return this;
        }

        public GoalCentricRequest withTXSupplier(DoubleSupplier tx){
            this.TxSupplier = tx;
            return this;
        }
    }

