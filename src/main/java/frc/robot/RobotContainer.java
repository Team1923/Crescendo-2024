package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.AutoUtils.AutoChooser;
import frc.lib.AutoUtils.AutoInstantiator;
import frc.lib.ShooterArmUtils.PositionRPMData;
import frc.robot.commands.Amp.EjectIntoAmp;
import frc.robot.commands.IndependentTesting.ActivateBeamBreak;
import frc.robot.commands.IndependentTesting.ArmMotionMagicCommandAMP;
import frc.robot.commands.IndependentTesting.ArmMotionMagicCommandShooter;
import frc.robot.commands.IndependentTesting.ArmPercOutCommand;
import frc.robot.commands.IndependentTesting.FeederPercOutCommand;
import frc.robot.commands.IndependentTesting.IntakeArmPercOutCommand;
import frc.robot.commands.IndependentTesting.IntakeMotionMagicCommand;
import frc.robot.commands.IndependentTesting.RunIntakeRollerCommand;
import frc.robot.commands.IndependentTesting.ShooterVelocityCommand;
import frc.robot.commands.Intake.DeployIntakeCommand;
import frc.robot.commands.Intake.IntakeEjectCommand;
import frc.robot.commands.Speaker.ScoreInSpeaker;
import frc.robot.commands.Swerve.AlignToAmp;
import frc.robot.commands.Swerve.GoalCentricCommand;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator= new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final int operatorLeftY = PS4Controller.Axis.kLeftY.value;
    private final int operatorRightY = PS4Controller.Axis.kRightY.value;

    /* Driver Buttons */
    private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);


    private final JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /*Beam Break Buttons */
    private final JoystickButton triangleButton = new JoystickButton(operator, 4);
    private final JoystickButton squareButton = new JoystickButton(operator, PS4Controller.Button.kSquare.value);
    private final JoystickButton circleButton = new JoystickButton(operator, 2);
    private final JoystickButton crossButton = new JoystickButton(operator, 1);


    /* Subsystems */
    // private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    // private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final FeederSubsystem feederSubsystem = new FeederSubsystem();

    /* Helper Classes */
    // private AutoInstantiator autoInstantiator = new AutoInstantiator();
    public final ShuffleboardSubsystem shuffleboardSubsystem = new ShuffleboardSubsystem();
    public final PositionRPMData positionRPMData = new PositionRPMData();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // s_Swerve.setDefaultCommand(
        //         new TeleopSwerve(
        //                 s_Swerve,
        //                 () -> -0.3*driver.getRawAxis(translationAxis),
        //                 () -> -0.3* driver.getRawAxis(strafeAxis),
        //                 () -> -0.3*driver.getRawAxis(rotationAxis),
        //                 () -> leftBumper.getAsBoolean()));

         //intakeSubsystem.setDefaultCommand(new IntakeArmPercOutCommand(intakeSubsystem, () -> 0.2 * operator.getRawAxis(operatorLeftY)));
        // armSubsystem.setDefaultCommand(new ArmPercOutCommand(armSubsystem, () -> 0.2 * operator.getRawAxis(operatorRightY)));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // yButton.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        circleButton.whileTrue(new ArmMotionMagicCommandShooter(armSubsystem));
        // crossButton.whileTrue(new ScoreInSpeaker());


        // triangleButton.whileTrue(new FeederPercOutCommand(feederSubsystem));
        // squareButton.whileTrue(new RunIntakeRollerCommand(intakeSubsystem));
        

        // rightBumper.whileTrue(new GoalCentricCommand(s_Swerve, 
        //         () -> -driver.getRawAxis(translationAxis),
        //         () -> -driver.getRawAxis(strafeAxis),
        //         () -> -driver.getRawAxis(rotationAxis)));

        //aButton.whileTrue(new AlignToAmp(s_Swerve, () -> -driver.getRawAxis(strafeAxis)));

        //SmartDashboard.putData(new DeployIntakeCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command initializeAuto(AutoChooser selector) {
        return null;
    }
}
