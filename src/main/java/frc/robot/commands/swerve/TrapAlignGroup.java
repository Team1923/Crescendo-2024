// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapAlignGroup extends SequentialCommandGroup {
  /** Creates a new TrapAlignGroup. */
  public TrapAlignGroup(CommandSwerveDrivetrain swerve, DoubleSupplier translation, DoubleSupplier strafe) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new LockHeadingToTrap(swerve, translation, strafe, LimelightSubsystem.getSeenTrapHeading()),
      new MoveInLineWithTrap(swerve, translation, LimelightSubsystem.getSeenTrapHeading()),
      AutoBuilder.pathfindToPose(getFinalPose(), 
        new PathConstraints(1.0, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
        0 , 0)
    );
  }


  private Pose2d getFinalPose(){
    Pose2d currentRobotPose = StateHandler.getInstance().getRobotPose();

    double distToTag = StateHandler.getInstance().getDistanceToTrapTag();

    double theta = LimelightSubsystem.getSeenTrapHeading();

    double xOffset = Math.cos(Math.toDegrees(theta)) * distToTag;

    double yOffset = Math.sin(Math.toDegrees(theta)) * distToTag;

    
    return new Pose2d(currentRobotPose.getX()+xOffset, currentRobotPose.getY()+yOffset, Rotation2d.fromDegrees(theta));


  }
}
