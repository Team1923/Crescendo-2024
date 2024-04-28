// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.CustomWidgets.MotorConfigWidget;
import frc.robot.lib.StateMachine.StateVariables;
import frc.robot.lib.StateMachine.StateVariables.SwerveRequests;

public class PIDWidgetSubsystem extends SubsystemBase {

  public ShuffleboardTab pidDashboard = Shuffleboard.getTab("PID");

  private PhoenixPIDController swerveHeadingPID = new PhoenixPIDController(SwerveConstants.headingKP, SwerveConstants.headingKI, SwerveConstants.headingKD);
  private ComplexWidget headingPIDWidget = Shuffleboard.getTab("PID").add("Heading Controller", swerveHeadingPID);


  /** Creates a new PIDSubsystem. */
  public PIDWidgetSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    StateVariables.applyPID((SwerveRequest.FieldCentricFacingAngle)SwerveRequests.FACING_AMP.request, swerveHeadingPID);
    StateVariables.applyPID((SwerveRequest.FieldCentricFacingAngle)SwerveRequests.GOAL_CENTRIC.request, swerveHeadingPID);


  }
}
