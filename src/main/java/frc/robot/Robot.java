// Copytop (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private TalonFX topmotor = new TalonFX(0);//find ids on phoenix tuner
  private TalonFX bottommotor = new TalonFX(1);
  private TalonFX feederMotor = new TalonFX(2);

  private Joystick control = new Joystick(0);


  //being run in RPM
  private final double velocitytop = 6000;
  private final double velocitybottom = 6000;


  private Timer offsetWait = new Timer();


  private final double RPMOFFSET = 200; //50 rpm every controller press to increase/decrease
  private double v_lOffset = 0;
  private double v_rOffset = 0;

                      
                  //max rpm/secs conversion to RPS
  //private final double kP = 0.4; //duty cycle per rps, if we are off by 6000 rps, use 100% speed

  private final VelocityVoltage m_velocitytop = new VelocityVoltage(0);
  private final VelocityVoltage m_velocitybottom = new VelocityVoltage(0);

  // private final VelocityDutyCycle m_velocitytop = new VelocityDutyCycle(0);
  // private final VelocityDutyCycle m_velocitybottom = new VelocityDutyCycle(0);


  //in RPM



  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


    offsetWait.reset();
    offsetWait.start();

    //for motor 1
    topmotor.getConfigurator().apply(new TalonFXConfiguration());
    

    //var config = motor1.getConfigurator();
     
    /**
     * This is for the BOTTOM Motor
     */
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; //0.05
    slot0Configs.kV = 0.11;
    slot0Configs.kP = 0.11; // old: 0.2, new: 0.40039100684261975
    slot0Configs.kI = 0; // 0
    slot0Configs.kD = 0; // old : 0.4, new: 0.0008007820136852395

    /**
     * We need to change this for TOP Motor
     */
    var slot1Configs = new Slot1Configs();
    slot1Configs.kS = 0;
    slot1Configs.kV = 0;
    slot1Configs.kP = 0;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0;

    // config.apply(slotConfigs);

    //for motor 2
     bottommotor.getConfigurator().apply(new TalonFXConfiguration());
     bottommotor.setInverted(false);
     topmotor.setInverted(false);

    // var config2 = motor2.getConfigurator();

    topmotor.getConfigurator().apply(slot0Configs, 0.05);
    bottommotor.getConfigurator().apply(slot0Configs, 0.05);
    // config2.apply(slotConfigs);

    feederMotor.getConfigurator().apply(new TalonFXConfiguration());

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


    if (offsetWait.hasElapsed(0.2)){
      if (control.getPOV(0)==0){
        v_lOffset+=RPMOFFSET;
      }
      if (control.getPOV()==180){
        v_lOffset-=RPMOFFSET;
      }

      //VERIFY FOR IN LAB CONTROLLER
      if (control.getRawButton(4)){
        v_rOffset+=RPMOFFSET;
      }
      if (control.getRawButton(1)){
        v_rOffset-=RPMOFFSET;
      }
      offsetWait.reset();
      offsetWait.start();
      
    }


    if(control.getRawButton(5)){
      m_velocitybottom.Slot = 0;
      m_velocitytop.Slot = 0;
      topmotor.setControl(m_velocitytop.withVelocity((velocitytop+v_rOffset)/60));//change
      bottommotor.setControl(m_velocitybottom.withVelocity((velocitybottom+v_lOffset)/60));//change 
      
      // topmotor.setControl(new VelocityDutyCycle(60));
      // motor2.setControl(new VelocityDutyCycle(velocity1/60));
    }
    else{
      topmotor.stopMotor();
      bottommotor.stopMotor();
    }

    if(control.getRawButton(1)){
      feederMotor.set(0.4);
    }
    else{
      feederMotor.stopMotor();
    }

    //times 60 to go to RPM
    SmartDashboard.putNumber("Velocity(RPM) of top motor", topmotor.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Velocity(RPM) of bottom motor", bottommotor.getVelocity().getValueAsDouble() * 60);

    SmartDashboard.putNumber("Set Velocity(RPM) for top motor", velocitytop+v_rOffset);
    SmartDashboard.putNumber("Set Velocity(RPM) for bottom motor", velocitybottom+v_lOffset);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
