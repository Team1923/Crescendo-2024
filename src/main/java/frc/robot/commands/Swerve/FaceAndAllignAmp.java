// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.LimelightUtil.LimelightInterface;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PosAndAllignAmp extends SequentialCommandGroup {

  LimelightInterface ll = LimelightInterface.getInstance();

  private final double tolerance = 1; //degree

  /** Creates a new PosAndAllignAmp. */
  public PosAndAllignAmp(SwerveSubsystem s, DoubleSupplier transSup, DoubleSupplier strafeSup, DoubleSupplier rotSup) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GoalCentricCommand(s, transSup, strafeSup, rotSup).onlyWhile(() -> isFacingAmp()),

      new AlignToAmp(s, strafeSup)

    );
  }


  public boolean isFacingAmp(){
    return ll.hasAmpTag() && (Math.abs(ll.getXAngleOffset()) < tolerance);
  }

}
