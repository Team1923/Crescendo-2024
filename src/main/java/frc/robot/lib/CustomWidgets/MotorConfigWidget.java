// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.CustomWidgets;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;


import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Add your docs here. */
public class MotorConfigWidget {

    private TalonFXConfiguration config;


    private PhoenixPIDController armPID = new PhoenixPIDController(0, 0, 0);;

    private ComplexWidget armPIDWidget;
    private PhoenixPIDController armVeloAccelJerk = new PhoenixPIDController(0, 0, 0);;

    private ComplexWidget armVeloAccelJerkWidget;



    public MotorConfigWidget(String motorName, TalonFXConfiguration startingConfig){
        /* Set up the motor to initially be in Brake mode. */
        config = startingConfig;

        armPID.setP(startingConfig.Slot0.kP);
        armPID.setI(startingConfig.Slot0.kI);
        armPID.setD(startingConfig.Slot0.kD);

        armVeloAccelJerk.setP(startingConfig.MotionMagic.MotionMagicCruiseVelocity);
        armVeloAccelJerk.setI(startingConfig.MotionMagic.MotionMagicAcceleration);
        armVeloAccelJerk.setD(startingConfig.MotionMagic.MotionMagicJerk);

        armPIDWidget = Shuffleboard.getTab("PID").add(motorName+" PID", armPID);
        armVeloAccelJerkWidget = Shuffleboard.getTab("PID").add(motorName+" Velo Accel Jerk", armVeloAccelJerk);


    }

    public TalonFXConfiguration updatedConfig(){

        config.Slot0.kP = armPID.getP();
        config.Slot0.kI = armPID.getI();
        config.Slot0.kD = armPID.getD();

        config.MotionMagic.MotionMagicCruiseVelocity = armVeloAccelJerk.getP();
        config.MotionMagic.MotionMagicAcceleration = armVeloAccelJerk.getI();
        config.MotionMagic.MotionMagicJerk = armVeloAccelJerk.getD();

        return config;
         
    }


    

}
