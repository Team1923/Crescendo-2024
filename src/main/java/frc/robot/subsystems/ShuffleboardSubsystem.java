// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {
  /** Creates a new ShuffleboardSubsystem. */

  public ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");

  private GenericEntry driverStationTimeElapsed = driverDashboard
    .add("Time Elapsed", DriverStation.getMatchTime())
    .withSize(2, 1)
    .withPosition(2, 2)
    .getEntry();

  @Override
  public void periodic() {
    driverStationTimeElapsed.setDouble(DriverStation.getMatchTime());
  }
}
