// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AllignToAmpBetter extends Command {
  /** Creates a new AllignToAmpBetter. */
  public AllignToAmpBetter() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  /**
   * 
   * @return determines if the robot is ready to start allignment, meaning that the limelight's x offset is representative of what direction the tag is relative to the field
   */
  public boolean isInGoodPosition(SwerveSubsystem s){


    //TODO: swap 90 and -90, blue should actually be -90, only using 90 because of where we put the apriltag in the hallway
    double finalHeading = DriverStation.getAlliance().get() == Alliance.Blue ? 90 : -90;


    //never ambiguous here
    if (Math.abs(finalHeading)>Math.abs(s.getGyroYaw().getDegrees())){
      return true;
    }
    

    
}
