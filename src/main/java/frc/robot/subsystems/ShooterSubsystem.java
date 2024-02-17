// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.RobotStateUtils.StateHandler;
import frc.lib.RobotStateUtils.StateVariables;
import frc.lib.RobotStateUtils.StateVariables.ShooterSpeeds;
import frc.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /* Instantiate Shooter Motors (top + bottom) */
  private TalonFX shooterTop = new TalonFX(ShooterConstants.shooterMotorPrimaryID, "rio");
  private TalonFX shooterBottom = new TalonFX(ShooterConstants.shooterMotorFollowerID, "rio");

  /* Instantiate DigitalInput object to get data from beam break. */
  private DigitalInput beamBreakFour = new DigitalInput(ShooterConstants.beamBreakFourID);

  /* Initialize StateHandler to get important data about the robot. */
  private StateHandler stateHandler = StateHandler.getInstance();

  /* Helper class used to get the RPM the shooter needs to operate at when shooting from range. */
  PositionRPMData rpmData = PositionRPMData.getInstance();

  /* Declare MotionMagicVoltage object to command the shooter at a specific velocity. */
  private final MotionMagicVelocityVoltage motionMagicVelVoltage;


  /* Construct a new ShooterSubsystem to apply motor configurations. */
  public ShooterSubsystem() {
    /* Create a new TalonFXConfiguration. Will be used to set relevant constants for the shooter. */
    var shooterFXConfig = new TalonFXConfiguration();

    /* Setup shooter motors to operate in BRAKE mode. */
    shooterFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /* Shooter motion constants applied to slot 0. TUNE THESE! */
    var shooterMotorConfig = shooterFXConfig.Slot0;
    shooterMotorConfig.kS = ShooterConstants.shooterKS;
    shooterMotorConfig.kV = ShooterConstants.shooterKV;
    shooterMotorConfig.kA = ShooterConstants.shooterKA;
    shooterMotorConfig.kP = ShooterConstants.shooterKP;
    shooterMotorConfig.kI = ShooterConstants.shooterKI;
    shooterMotorConfig.kD = ShooterConstants.shooterKD;

    /* Initialize MotionMagicVoltage object to apply motion magic on the shooter. */
    motionMagicVelVoltage = new MotionMagicVelocityVoltage(0);
    motionMagicVelVoltage.Slot = 0;

    /* Get the MotionMagic configuration to specify acceleration and jerk. */
    var shooterMotionMagicConfig = shooterFXConfig.MotionMagic;
    shooterMotionMagicConfig.MotionMagicAcceleration = ShooterConstants.maxShooterAccel; //TODO: Tune
    shooterMotionMagicConfig.MotionMagicJerk = ShooterConstants.maxShooterJerk; //TODO: Tune

    /* Apply configuration to each of the shooter motors. */
    shooterTop.getConfigurator().apply(shooterFXConfig, 0.05);
    shooterBottom.getConfigurator().apply(shooterFXConfig, 0.05);

    /* Set one motor to follow the other. */
    shooterBottom.setControl(new Follower(ShooterConstants.shooterMotorPrimaryID, false));
  }

  /**
   * Method to set speed of shooter motors, using velocity closed loop control.
   * 
   * @param velocityP The speed, in RPM, passed into the top motor.
   * @param velocityF The speed, in RPM, passed into the bottom motor.
   */
  public void setVelocities(double velocityP, double velocityF) {
    shooterTop.setControl(motionMagicVelVoltage.withVelocity(velocityP * ShooterConstants.shooterRPMToRPS));
    // if (velocityF == 0) {
    //   shooterBottom.setControl(motionMagicVelVoltage.withVelocity(velocityF * ShooterConstants.shooterRPMToRPS));
    // } else {
    //   shooterBottom.setControl(motionMagicVelVoltage.withVelocity((velocityF - 400) * ShooterConstants.shooterRPMToRPS));
    // }
    shooterBottom.setControl(motionMagicVelVoltage.withVelocity((velocityF) * ShooterConstants.shooterRPMToRPS));
  }

  /**
   * Method to get the digital input reading of BB4.   * 
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakFour() {
    return !beamBreakFour.get();
  }

  /**
   * Returns the velocity in RPM of the top shooter motor.
   * 
   * @return Velocity in RPM.
   */
  public double getTopRPM() {
    return shooterTop.getVelocity().getValueAsDouble() * ShooterConstants.shooterRPSToRPM;
  }

  /**
   * Returns the velocity in RPM of the bottom shooter motor.
   * 
   * @return Velocity in RPM.
   */
  public double getBottomRPM() {
    return shooterBottom.getVelocity().getValueAsDouble() * 60;
  }

  /**
   * Stops the motors.
   */
  public void stopMotors() {
    shooterTop.set(0);
    shooterBottom.set(0);
  }

  /**
   * Method to determine if the current shooter RPM is the commanded shooter RPM.
   * @param desiredSetpoint the commanded velocity setpoint to check/analyze.
   * @return a boolean to determine if the shooter's velocity is within range of the commanded velocity.
   */
  public boolean isAtShooterSpeed(double desiredSetpoint) {
    return Math.abs(getTopRPM() - desiredSetpoint) < ShooterConstants.shooterSpeedThreshold
        && Math.abs(getBottomRPM() - (desiredSetpoint)) < ShooterConstants.shooterSpeedThreshold;
  }

  @Override
  public void periodic() {
    stateHandler.setBBFourCovered(getBeamBreakFour());

    // SmartDashboard.putNumber("Raw RPS TOP SHOOTER", shooterTop.getVelocity().getValueAsDouble());
    // SmartDashboard.putNumber("Raw RPS BOTTOM SHOOTER", shooterBottom.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("RPM TOP SHOOTER", getTopRPM());
    SmartDashboard.putNumber("RPM BOTTOM SHOOTER", getBottomRPM());

    ShooterSpeeds desiredShooterSpeedState = stateHandler.getDesiredShootingSpeed();
    double desiredShooterSpeed = desiredShooterSpeedState.getRPMValue().getRPM();

    if (desiredShooterSpeedState == ShooterSpeeds.SHOOT) {
      /* If at subwoofer, then the desired shot speed is the preset for the subwoofer shot. */
      if (stateHandler.getWantToPositionForSubwoofer()) {
        desiredShooterSpeed = ShooterSpeeds.SHOOT.getRPMValue().getRPM();
      }
      /* If we have a valid tag, then get positional data. */
      else if (stateHandler.getHasValidSpeakerTag()) {
        desiredShooterSpeed = rpmData.getDesiredShooterRPM(stateHandler.getDistanceToSpeakerTag());
      }
    } 

    /* Set the desired velocity of the shooter wheels. */
    setVelocities(desiredShooterSpeed, desiredShooterSpeed);


    if (isAtShooterSpeed(desiredShooterSpeed)) {
      stateHandler.setCurrentShootingSpeed(desiredShooterSpeedState);
    }

  }
}