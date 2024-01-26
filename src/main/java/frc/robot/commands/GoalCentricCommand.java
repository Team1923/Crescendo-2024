// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightUtil.LimelightInterface;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class GoalCentricCommand extends Command {
  private final SwerveSubsystem s_Swerve;
  private final LimelightInterface limelight = LimelightInterface.getInstance();

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

  private final double kPTarget = 0.5/Constants.LimeLightConstants.limelightViewingAngle; // TODO: will most likley need to change, right now is half speed (0.5) / max angle

  /** Creates a new GoalCentricCommand. */
  public GoalCentricCommand(SwerveSubsystem s, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
   this.s_Swerve = s;
   this.translationSup = translationSup;
   this.strafeSup = strafeSup;
   this.rotationSup = rotationSup;

    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double translationVal = translationSup.getAsDouble();
   double strafeVal = strafeSup.getAsDouble();

   double rotationVal = 0;

   if(Math.abs(rotationSup.getAsDouble()) > 0.5){
    rotationVal = rotationSup.getAsDouble();
   } else if(limelight.hasSpeakerTag()){
      rotationVal = limelight.getXAngleOffset() * kPTarget;
   }
   else{
    rotationVal = 0;
   }

   translationVal = Math.abs(translationVal) > Swerve.stickDeadband? translationVal : 0.0;
   strafeVal = Math.abs(strafeVal) > Swerve.stickDeadband ? strafeVal : 0.0;
   rotationVal = Math.abs(rotationVal) > Swerve.stickDeadband ? rotationVal : 0.0;

   if(DriverStation.getAlliance().get() == Alliance.Blue){
            s_Swerve.drive(new Translation2d(translationVal,strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal*Constants.Swerve.maxAngularVelocity, !robotCentricSup.getAsBoolean(), true);
        }
        else{
             s_Swerve.drive(new Translation2d(translationVal,strafeVal).times(-Constants.Swerve.maxSpeed), 
            rotationVal*Constants.Swerve.maxAngularVelocity, !robotCentricSup.getAsBoolean(), true);
        }

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
