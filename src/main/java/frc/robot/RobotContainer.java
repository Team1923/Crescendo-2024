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
import frc.robot.commands.Climb.ClimbCommandGroup;
import frc.robot.commands.ManualHashTuningCommands.ChangePositionOffset;
import frc.robot.commands.ManualHashTuningCommands.ChangeRPMOffset;
import frc.robot.commands.desired_scoring_location.SetArmToAmp;
import frc.robot.commands.desired_scoring_location.SetArmToFrontAmp;
import frc.robot.commands.desired_scoring_location.SetArmToPunt;
import frc.robot.commands.desired_scoring_location.SetArmToRanged;
import frc.robot.commands.desired_scoring_location.SetArmToReverseSubwoofer;
import frc.robot.commands.desired_scoring_location.SetArmToSubwoofer;
import frc.robot.commands.desired_scoring_location.SetArmToTrap;
import frc.robot.commands.desired_scoring_location.SetArmToUnguardable;
import frc.robot.commands.intake.BabyBird;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.FullEjectCommand;
import frc.robot.commands.intake.IntakeEjectCommand;
import frc.robot.commands.intake.PuntShot;
import frc.robot.commands.scoring.GCScoreCommandGroup;
import frc.robot.commands.scoring.ScoreGamePiece;
import frc.robot.commands.scoring.ScoreGamePieceNoRanged;
import frc.robot.commands.swerve.Align90;
import frc.robot.commands.swerve.AlignAmp;
import frc.robot.commands.swerve.AlignCommandGroup;
import frc.robot.commands.swerve.AlignHeading;
import frc.robot.commands.swerve.FaceAndAlignToAmp;
import frc.robot.commands.swerve.GoalCentricCommand;
import frc.robot.commands.swerve.LockHeadingToTrap;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.Autonomous.AutoChooser;
import frc.robot.lib.Autonomous.AutoInstantiator;
import frc.robot.lib.Controller.ControllerLimiter;
import frc.robot.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.InfoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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
  public final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  /* Helper Classes Instantiation */
  public final AutoInstantiator autoInstantiator = new AutoInstantiator();
  public final PositionRPMData positionRPMData = new PositionRPMData();
  private final InfoSubsystem infoSubsystem = new InfoSubsystem(driverXboxController, operatorPS4Controller);
  //TODO: Add InfoSubsystem!

  /* Simulation telemetry utility - makes simulating swerve easier. */
  private final Telemetry logger = new Telemetry(SwerveConstants.maxSpeed);

  /* Swerve Controller Inversion Arrays */
  public static final int[] blueJoystickValues = {-1, -1, -1};
  public static final int[] redJoystickValues = {1, 1, -1};

  /* Separate method to configure all the button bindings. */
  private void configureBindings() {
    /* Default Swerve Drive Command */
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(getSwerveJoystickInput()[0] 
          * driverXboxController.getLeftY() * SwerveConstants.maxSpeed)
          .withVelocityY(getSwerveJoystickInput()[1] * driverXboxController.getLeftX() * SwerveConstants.maxSpeed)
          .withRotationalRate(getSwerveJoystickInput()[2] * ControllerLimiter.quadratic(driverXboxController.getRightX()*0.6) * SwerveConstants.maxAngularRate)));
    //getSwerveJoystickInput()[2] * ControllerLimiter.quadratic(driverXboxController.getRightX()) * SwerveConstants.maxAngularRate

    /* Zero the Gyro when pressing Y on the XBOX Controller */
    driverXboxController.button(ControllerConstants.Driver.yButton).onTrue(drivetrain.runOnce(() -> drivetrain.zeroGyro()));

    // driverXboxController.rightStick().whileTrue(new Align90(drivetrain, () -> getSwerveJoystickInput()[0]*driverXboxController.getLeftY(), () -> getSwerveJoystickInput()[1]*driverXboxController.getLeftX()));
    driverXboxController.rightStick().whileTrue(new AlignHeading(drivetrain, () -> getSwerveJoystickInput()[0]*driverXboxController.getLeftY(), () -> getSwerveJoystickInput()[1]*driverXboxController.getLeftX()));

    /* Simulation tool for Swerve */
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    /* Driver Button Bindings */
    //TODO: add "ScoreCommandGroup" here!
    // driverXboxController.rightTrigger().whileTrue(new ScoreGamePiece());
   driverXboxController.leftTrigger().whileTrue(new GCScoreCommandGroup(drivetrain,
    () -> getSwerveJoystickInput()[0] * driverXboxController.getLeftY(), 
    () -> getSwerveJoystickInput()[1] * driverXboxController.getLeftX(), 
    () -> getSwerveJoystickInput()[2] * driverXboxController.getRightX()));

    driverXboxController.rightTrigger().whileTrue(new ScoreGamePieceNoRanged());

    /* manual hashmap tuning */
    // operatorPS4Controller.povUp().whileTrue(new ChangeRPMOffset(25));
    // operatorPS4Controller.povDown().whileTrue(new ChangeRPMOffset(-25));

    // driverXboxController.povUp().whileTrue(new ChangePositionOffset(0.0025));
    // driverXboxController.povDown().whileTrue(new ChangePositionOffset(-0.0025));




    /* Operator Button Bindings */
    //TODO: add buttons and IDs to constants
    operatorPS4Controller.button(ControllerConstants.Operator.triangleButton).onTrue(new SetArmToRanged());
    operatorPS4Controller.button(ControllerConstants.Operator.squareButton).onTrue(new SetArmToFrontAmp());
    operatorPS4Controller.button(ControllerConstants.Operator.crossButton).onTrue(new SetArmToSubwoofer());
    operatorPS4Controller.button(Constants.ControllerConstants.Operator.circleButton).onTrue(new SetArmToReverseSubwoofer());
    operatorPS4Controller.button(Constants.ControllerConstants.Operator.operatorRightTrigger).whileTrue(new PuntShot());
    operatorPS4Controller.povUp().onTrue(new SetArmToTrap());
    
    
    
    //TODO: add intake commands here!
    operatorPS4Controller.button(ControllerConstants.Operator.operatorRightBumper).whileTrue(new DeployIntakeCommand());
    operatorPS4Controller.button(ControllerConstants.Operator.operatorLeftBumper).whileTrue(new IntakeEjectCommand());

    //TODO: implement arm stuff
    operatorPS4Controller.button(ControllerConstants.Operator.littleRightButton).toggleOnTrue(new ClimbCommandGroup(armSubsystem, () -> operatorPS4Controller.getRawAxis(5)));

    //FULLEJECT 
    operatorPS4Controller.button(ControllerConstants.Operator.littleLeftButton).whileTrue(new FullEjectCommand());
    operatorPS4Controller.povLeft().whileTrue(new BabyBird());
    operatorPS4Controller.povRight().onTrue(new SetArmToUnguardable());

  }

  /* Helper method that does some inversion based on the alliance color. */
  public static int[] getSwerveJoystickInput() {
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
