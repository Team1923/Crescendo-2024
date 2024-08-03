// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.CustomWidgets;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class PIDWidget {

    private ShuffleboardTab tab;
    private TalonFX[] motors;

    private TalonFXConfiguration config;

    private GenericEntry PValue;
    private GenericEntry IValue;
    private GenericEntry DValue;
    private GenericEntry FFValue;

    private GenericEntry VValue;
    private GenericEntry AValue;
    private GenericEntry JValue;

    private SuppliedValueWidget<Double> position;
    private SuppliedValueWidget<Double> velocity;


    /**
     * 
     * @param name
     * @param motors
     * @param defaultConfig ASSUMES ALL MOTORS HAVE SAME DEFAULT CONFIG
     */
    public PIDWidget(String name, TalonFXConfiguration defaultConfig, double positionConversion, double velocityConversion, TalonFX... motors){
        tab = Shuffleboard.getTab(name);

        this.motors = motors;

        this.config = defaultConfig;


        PValue = tab.add("P", defaultConfig.Slot0.kP).withPosition(0, 0).getEntry();
        IValue = tab.add("I", defaultConfig.Slot0.kI).withPosition(1, 0).getEntry();
        DValue = tab.add("D", defaultConfig.Slot0.kD).withPosition(2,0).getEntry();
        FFValue= tab.add("FF (kS)", defaultConfig.Slot0.kS).withPosition(3, 0).getEntry();
        
        VValue = tab.add("Cruise Velocity", defaultConfig.MotionMagic.MotionMagicCruiseVelocity).withPosition(0, 1).getEntry();
        AValue = tab.add("Acceleration", defaultConfig.MotionMagic.MotionMagicAcceleration).withPosition(1, 1).getEntry();
        JValue = tab.add("Jerk", defaultConfig.MotionMagic.MotionMagicJerk).withPosition(2, 1).getEntry();

        position = tab.addDouble("CURRENT POSITION", () -> motors[0].getPosition().getValueAsDouble() * positionConversion).withPosition(4,0).withSize(2, 1);
        velocity= tab.addDouble("CURRENT VELOCITY", () -> motors[0].getVelocity().getValueAsDouble() * velocityConversion).withPosition(6,0).withSize(2, 1);

        tab.add("UPDATE", new InstantCommand(() -> updateMotor()).ignoringDisable(true)).withPosition(4, 1).withSize(2, 1);
    }

    public void updateMotor(){
        config.Slot0.kP = PValue.get().getDouble();
        config.Slot0.kI = IValue.get().getDouble();
        config.Slot0.kD = DValue.get().getDouble();
        config.Slot0.kS = FFValue.get().getDouble();

        config.MotionMagic.MotionMagicCruiseVelocity = VValue.get().getDouble();
        config.MotionMagic.MotionMagicAcceleration = AValue.get().getDouble();
        config.MotionMagic.MotionMagicJerk = JValue.get().getDouble();

        for (TalonFX motor : motors){
            motor.getConfigurator().apply(config);
        }


    }

}
