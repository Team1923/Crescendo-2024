// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.Limelight.LimelightInterface;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/* TODO: cross check with existing code to get state handler integration going */
public class GoalCentricCommand extends Command {
  /* Swerve + Limelight Initialization */
  private CommandSwerveDrivetrain swerve;
  private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.maxSpeed * 0.1).withRotationalDeadband(SwerveConstants.maxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private LimelightInterface limelight = LimelightInterface.getInstance();
  private StateHandler stateHandler = StateHandler.getInstance();

  /* Fancy double stuff for input + output to swerve. */
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  /* PID Things */
  private final double kPTarget = 0.01;
  private PIDController rotationController;

  /** Creates a new GoalCentricCommand. */
  public GoalCentricCommand(CommandSwerveDrivetrain swerve, DoubleSupplier t, DoubleSupplier s, DoubleSupplier r) {
    this.swerve = swerve;
    this.translationSup = t;
    this.strafeSup = s;
    this.rotationSup = r;
    rotationController = new PIDController(kPTarget, 0, 0);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationValue = translationSup.getAsDouble();
    double strafeValue = strafeSup.getAsDouble();
    
    swerve.applyRequest(() -> drive.withVelocityX(translationValue)
      .withVelocityY(strafeValue)
      .withRotationalRate(calculateRotationalValue()));
  }

  private double calculateRotationalValue() {
    /* There are 3 conditions for the rotational value:
     * - Driver inputs a very large rotational value
     * - The limelight has a valid tag, and the rotational speed is based on the angle to the tag.
     * - Neither is true, which means that the rotational velocity is 0 (the rotation is preserved).
     */
    if (Math.abs(rotationSup.getAsDouble()) > 0.5) {
      return rotationSup.getAsDouble();
    } else if (limelight.hasSpeakerTag()) {
      return rotationController.calculate(limelight.getXAngleOffset(), 0);
    } else {
      return 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.getWantToPositionForSubwoofer() || stateHandler.getScoreInAmp();
  }
}
