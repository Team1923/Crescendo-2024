// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.CustomWidgets;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.StateMachine.StateVariables.SwerveRequests;

/** Add your docs here. */
public class SwerveRequestConfigWidget {


    
private PhoenixPIDController swerveHeadingPID;
  private ComplexWidget headingPIDWidget;

  public SwerveRequestConfigWidget(SwerveRequests requestEnum){

    swerveHeadingPID = ((SwerveRequest.FieldCentricFacingAngle)requestEnum.request).HeadingController;

   headingPIDWidget = Shuffleboard.getTab("PID").add(requestEnum.toString()+" Heading Controller", swerveHeadingPID);
  }

  
}
