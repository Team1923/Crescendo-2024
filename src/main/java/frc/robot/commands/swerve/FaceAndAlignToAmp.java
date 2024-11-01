// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.Limelight.LimelightInterface;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FaceAndAlignToAmp extends SequentialCommandGroup {

  LimelightInterface ll = LimelightInterface.getInstance();

  private final double tolerance = 1; //degree

  /** Creates a new PosAndAllignAmp. */
  public FaceAndAlignToAmp(CommandSwerveDrivetrain s, DoubleSupplier transSup, DoubleSupplier strafeSup) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Align90(s,transSup,strafeSup),
      new AlignAmp(s, strafeSup)
      

    );
  }

  
}