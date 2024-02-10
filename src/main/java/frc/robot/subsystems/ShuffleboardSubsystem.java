// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateUtils.StateHandler;

public class ShuffleboardSubsystem extends SubsystemBase {
  /** Creates a new ShuffleboardSubsystem. */

  public ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
  private StateHandler stateHandler = StateHandler.getInstance();

  private GenericEntry driverStationTimeElapsed = driverDashboard
    .add("Time Elapsed", DriverStation.getMatchTime())
    .withSize(2, 1)
    .withPosition(2, 2)
    .getEntry();

  @Override
  public void periodic() {
    driverStationTimeElapsed.setDouble(DriverStation.getMatchTime());

    /* BEAM BREAK VALUES */
    SmartDashboard.putBoolean("BB ONE COVERED", stateHandler.getBBOneCovered());
    SmartDashboard.putBoolean("BB TWO COVERED", stateHandler.getBBTwoCovered());
    SmartDashboard.putBoolean("BB THREE COVERED", stateHandler.getBBThreeCovered());
    SmartDashboard.putBoolean("BB FOUR COVERED", stateHandler.getBBFourCovered());

    /* RELEVANT INTAKE STATES */
    SmartDashboard.putString("CURRENT INTAKE ROLLER", stateHandler.getCurrentIntakeRollerSpeed().toString());
    SmartDashboard.putString("DESIRED INTAKE POS", stateHandler.getDesiredIntakeState().toString());
    SmartDashboard.putString("CURRENT INTAKE POS", stateHandler.getCurrentIntakeState().toString());

    /* RELEVANT ARM STATES */
    SmartDashboard.putString("DESIRED ARM STATE", stateHandler.getDesiredArmState().toString());
    SmartDashboard.putString("CURRENT ARM STATE", stateHandler.getCurrentArmState().toString());

    /* RELEVANT SHOOTER STATES */
    SmartDashboard.putString("DESIRED SHOOTER STATE", stateHandler.getDesiredShootingSpeed().toString());
    SmartDashboard.putString("CURRENT SHOOTER STATE", stateHandler.getCurrentShootingSpeed().toString());

    /* RELEVANT FEEDER STATES */
    SmartDashboard.putString("CURRENT FEEDER DIRECTION", stateHandler.getCurrentFeederSpeed().toString());

    SmartDashboard.putBoolean("AMP POSITION", stateHandler.getScoreInAmp());
    SmartDashboard.putBoolean("SUBWOOFER POSITION", stateHandler.getWantToPositionForSubwoofer());
  }
}
