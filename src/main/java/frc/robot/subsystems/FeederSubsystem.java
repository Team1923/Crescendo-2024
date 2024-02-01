// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateVariables.ArmStates;
import frc.lib.RobotStateUtils.StateVariables.FeederSpeeds;
import frc.lib.RobotStateUtils.StateVariables.IntakeRollerSpeeds;
import frc.lib.RobotStateUtils.StateVariables.IntakeStates;
import frc.lib.RobotStateUtils.StateVariables.ShooterSpeeds;
import frc.robot.Constants.FeederConstants;
import frc.lib.RobotStateUtils.StateHandler;

public class FeederSubsystem extends SubsystemBase {
  private DigitalInput beamBreakTwo = new DigitalInput(FeederConstants.beamBreakTwoID);
  private DigitalInput beamBreakThree = new DigitalInput(FeederConstants.beamBreakThreeID);
  private StateHandler stateHandler = StateHandler.getInstance();

  private TalonFX feederMotor = new TalonFX(FeederConstants.feederMotorID);

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    feederMotor.getConfigurator().apply(new TalonFXConfiguration());
  }

  /**
   * This method sets the speed of the feeder motor wheel.
   * 
   * @param s The desired output speed.
   */
  public void setFeederMotorSpeed(double s) {
    feederMotor.set(s);
  }

  /**
   * This method stops the motor.
   */
  public void stopFeederMotor() {
    feederMotor.stopMotor();
  }

  /**
   * Method to get the digital input reading of BB2.
   * TODO: verify if adding a NOT before the boolean is needed.
   * 
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakTwo() {
    return beamBreakTwo.get();
  }

  /**
   * Method to get the digital input reading of BB3.
   * TODO: verify if adding a NOT before the boolean is needed.
   * 
   * @return the boolean value representing the digital input reading.
   */
  public boolean getBeamBreakThree() {
    return beamBreakThree.get();
  }

  @Override
  public void periodic() {
    stateHandler.setBBTwoCovered(getBeamBreakTwo());
    stateHandler.setBBThreeCovered(getBeamBreakThree());

    FeederSpeeds desiredFeederSpeed = StateHandler.getInstance().getDesiredFeederSpeed();

    /*
     * CONDITION: Want to eject game piece.
     */
    if (stateHandler.getCurrentIntakeState() == IntakeStates.DEPLOYED
        && stateHandler.getCurrentIntakeRollerSpeed() == IntakeRollerSpeeds.EJECT) {
      desiredFeederSpeed = FeederSpeeds.OUTWARD;
    }

    /*
     * CONDITION: Ready to score into speaker.
     * NOTE: the centeredToTag method will always return true on default.
     */
    else if (stateHandler.getCurrentArmState() == ArmStates.SPEAKER
        && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT
        && stateHandler.getIsCenteredToTag()) {
      desiredFeederSpeed = FeederSpeeds.INWARD;
    }


    setFeederMotorSpeed(desiredFeederSpeed.getPercentOutputValue().getPercentOut());
    stateHandler.setCurrentFeederSpeed(desiredFeederSpeed);

  }

}