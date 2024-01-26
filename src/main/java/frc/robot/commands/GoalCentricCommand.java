// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightUtil.LimelightInterface;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class GoalCentricCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final LimelightInterface limelight = LimelightInterface.getInstance();

  private final DoubleSupplier translationSpeedSupplier, strafeSpeedSupplier, rotationSpeedSupplier;

  private final double kPTarget = 0.15; // will most likley need to change 

  /** Creates a new GoalCentricCommand. */
  public GoalCentricCommand(SwerveSubsystem s, DoubleSupplier xS, DoubleSupplier yS, DoubleSupplier tS) {
   this.swerveSubsystem = s;
   this.translationSpeedSupplier = xS;
   this.strafeSpeedSupplier = yS;
   this.rotationSpeedSupplier = tS;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double translationSpeed = translationSpeedSupplier.getAsDouble();
   double strafeSpeed = strafeSpeedSupplier.getAsDouble();

   double inputRotational = rotationSpeedSupplier.getAsDouble();

   double turningSpeed;
   if(Math.abs(inputRotational) > 0.5){
    turningSpeed = inputRotational;
   } else if(limelight.hasSpeakerTag()){
      turningSpeed = limelight.getXAngleOffset() * kPTarget;
   }
   else{
    turningSpeed = 0;
   }

   translationSpeed = Math.abs(translationSpeed) > Swerve.stickDeadband? translationSpeed : 0.0;
   strafeSpeed = Math.abs(strafeSpeed) > Swerve.stickDeadband ? strafeSpeed : 0.0;
   turningSpeed = Math.abs(turningSpeed) > Swerve.stickDeadband ? turningSpeed : 0.0;

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
