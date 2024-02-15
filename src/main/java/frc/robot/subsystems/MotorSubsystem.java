// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
  /** Creates a new MotorSubsystem. */

  TalonFX kraken = new TalonFX(0);

  private VelocityVoltage m_VeloVolt;

  public MotorSubsystem() {
      kraken.getConfigurator().apply(new TalonFXConfiguration());

      //NOTE: These configs were found for TALONS
      var slot0Configs = new Slot0Configs();
      slot0Configs.kS = 0; //0 
      slot0Configs.kV = 0.11; //0.11
      slot0Configs.kP = 0.4; // 0.5
      slot0Configs.kI = 0.001; // 0.001
      slot0Configs.kD = 0; //0

      m_VeloVolt = new VelocityVoltage(0);
      m_VeloVolt.Slot = 0;

      kraken.getConfigurator().apply(slot0Configs);
  }

  public void setPercOut(double out){
    kraken.set(out);
  }

  public void setVelocity(double rpm){
    kraken.setControl(new VelocityVoltage(rpm/60.0));
  }

  public void stop(){
    kraken.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("RAW MOTOR VELO (RPS)", kraken.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("MOTOR VELO (RPM)", kraken.getVelocity().getValueAsDouble() * 60);

  }
}
