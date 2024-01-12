// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleOpCommands.TeleopSwerve;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* SUBSYSTEM Initializations */
    public final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    

    /* XBOX Controller Setup */
    private final Joystick driver = new Joystick(0);
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final JoystickButton xBoxRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton xBoxLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton xBoxAButton = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton xBoxBButton = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton xBoxXButton = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton xBoxYButton = new JoystickButton(driver, XboxController.Button.kY.value);  


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands();
  }


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
  }

  private void setDefaultCommands() {
        s_Swerve.setDefaultCommand(
          new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> xBoxLeftBumper.getAsBoolean(),
            () -> xBoxRightBumper.getAsBoolean()));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Test_Auto");
  }
}
