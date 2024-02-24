// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Intake.DeployIntakeCommand;
import frc.robot.commands.Intake.IntakeEjectCommand;
import frc.robot.commands.Scoring.ScoreCommandGroup;
import frc.robot.commands.desired_scoring_location.SetArmToAmp;
import frc.robot.commands.desired_scoring_location.SetArmToRanged;
import frc.robot.commands.desired_scoring_location.SetArmToSubwoofer;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.Autonomous.AutoChooser;
import frc.robot.lib.Autonomous.AutoInstantiator;
import frc.robot.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.InfoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  /* Controller Instantiations */
  private final CommandXboxController driverXboxController = new CommandXboxController(0);
  private final CommandPS4Controller operatorPS4Controller = new CommandPS4Controller(1);

  /* Subsystem Instantiations */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.maxSpeed * 0.1).withRotationalDeadband(SwerveConstants.maxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  /* Helper Classes Instantiation */
  public final AutoInstantiator autoInstantiator = new AutoInstantiator();
  public final PositionRPMData positionRPMData = new PositionRPMData();
  private final InfoSubsystem infoSubsystem = new InfoSubsystem(driverXboxController, operatorPS4Controller);
  //TODO: Add InfoSubsystem!

  /* Simulation telemetry utility - makes simulating swerve easier. */
  private final Telemetry logger = new Telemetry(SwerveConstants.maxSpeed);

  /* Swerve Controller Inversion Arrays */
    int[] blueJoystickValues = {-1, -1, -1};
    int[] redJoystickValues = {1, 1, -1};

  /* Separate method to configure all the button bindings. */
  private void configureBindings() {
    /* Default Swerve Drive Command */
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(getSwerveJoystickInput()[0] 
          * driverXboxController.getLeftY() * SwerveConstants.maxSpeed)
          .withVelocityY(getSwerveJoystickInput()[1] * driverXboxController.getLeftX() * SwerveConstants.maxSpeed)
          .withRotationalRate(getSwerveJoystickInput()[2] * driverXboxController.getRawAxis(2) * SwerveConstants.maxAngularRate)));

    /* Zero the Gyro when pressing Y on the XBOX Controller */
    driverXboxController.button(ControllerConstants.Driver.yButton).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
  

    /* Simulation tool for Swerve */
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    /* Driver Button Bindings */
    //TODO: add "ScoreCommandGroup" here!
   driverXboxController.rightTrigger().whileTrue(new ScoreCommandGroup(drivetrain,
    () -> getSwerveJoystickInput()[0] * driverXboxController.getLeftY(), 
    () -> getSwerveJoystickInput()[1] * driverXboxController.getLeftX(), 
    () -> getSwerveJoystickInput()[2] * driverXboxController.getRightX()));

    /* Operator Button Bindings */
    //TODO: add buttons and IDs to constants
    operatorPS4Controller.button(ControllerConstants.Operator.triangleButton).onTrue(new SetArmToRanged());
    operatorPS4Controller.button(ControllerConstants.Operator.squareButton).onTrue(new SetArmToAmp());
    operatorPS4Controller.button(ControllerConstants.Operator.crossButton).onTrue(new SetArmToSubwoofer());

    //TODO: add intake commands here!
    operatorPS4Controller.button(ControllerConstants.Operator.operatorRightBumper).whileTrue(new DeployIntakeCommand());
    operatorPS4Controller.button(ControllerConstants.Operator.operatorLeftBumper).whileTrue(new IntakeEjectCommand());

  }

  /* Helper method that does some inversion based on the alliance color. */
  public int[] getSwerveJoystickInput() {
    /* 
     * Index 0: driverXboxController.getLeftY();
     * Index 1: driverXboxController.getLeftX();
     * Index 2: driverXboxController.getRightX();
     */
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return redJoystickValues;
    } else {
      return blueJoystickValues;
    }
  }

  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command initializeAuto(AutoChooser selector) {
    return selector.startMode();
  }
}
