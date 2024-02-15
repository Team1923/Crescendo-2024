// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;


import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightUtil.LimelightInterface;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class GoalCentricCommand extends Command {
  private final SwerveSubsystem s_Swerve;
  private final LimelightInterface limelight = LimelightInterface.getInstance();

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private SlewRateLimiter translateLimiter, strafeLimiter, rotateLimiter;

  private final double kPTarget = 0.004; // Tune this by yourself.
  private PIDController target;

  /** Creates a new GoalCentricCommand. */
  public GoalCentricCommand(SwerveSubsystem s, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    this.s_Swerve = s;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.translateLimiter = new SlewRateLimiter(Swerve.maxAccel);
    this.strafeLimiter = new SlewRateLimiter(Swerve.maxAccel);
    this.rotateLimiter = new SlewRateLimiter(Swerve.maxAngularAccel);
    

    target =  new PIDController(kPTarget, 0, 0);

    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationVal = translationSup.getAsDouble();
    double strafeVal = strafeSup.getAsDouble();

    double rotationVal = Math.abs(rotationSup.getAsDouble()) > 0.1 ? rotationSup.getAsDouble() : 0.0;

    
    if (limelight.hasSpeakerTag()) {
      rotationVal = target.calculate(limelight.getXAngleOffset(), 0);
    } 
    // else if (Math.abs(rotationSup.getAsDouble()) > 0.5) {
    //   rotationVal = rotationSup.getAsDouble();
    // else {
    //   rotationVal = 0;
    // }

    translationVal = Math.abs(translationVal) > Swerve.stickDeadband ? translationVal : 0.0;
    strafeVal = Math.abs(strafeVal) > Swerve.stickDeadband ? strafeVal : 0.0;
    // rotationVal = Math.abs(rotationVal) > 0.01 ? rotationVal : 0.0;

    translationVal = translateLimiter.calculate(translationVal) * Swerve.maxSpeed;
    strafeVal = strafeLimiter.calculate(strafeVal) * Swerve.maxSpeed;
    rotationVal = MathUtil.applyDeadband(rotationVal,0.005) * Swerve.maxAngularVelocity;

    ChassisSpeeds chassisSpeeds;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translationVal, strafeVal, rotationVal, s_Swerve.getGyroYaw());
    } else {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-translationVal, -strafeVal, rotationVal, s_Swerve.getGyroYaw());
    }

    

    SwerveModuleState[] moduleStates = Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    s_Swerve.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
