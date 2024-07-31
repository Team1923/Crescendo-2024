// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommand.AutoIntake;
import frc.robot.commands.AutoCommand.AutoScoreCommandGroup;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.SwerveRequests;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourceSideAuto extends SequentialCommandGroup {

  PathConstraints constraints = new PathConstraints(4.5, 3, Math.toRadians(720), Math.toRadians(540));

  /** Creates a new SourceSideAuto. */
  public SourceSideAuto(CommandSwerveDrivetrain swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    StateHandler.getInstance().setCurrentSwerveRequest(SwerveRequests.AUTO);
    addCommands(
      new PathPlannerAuto("Opening"),
      new AutoScoreCommandGroup(swerve),
      new ParallelDeadlineGroup(
          AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ShootTo5"), constraints),
          new AutoIntake()
      ),
      new ParallelDeadlineGroup(
        new NoteSearch(swerve, NoteSearch.Direction.LEFT),
        new AutoIntake()
      ).onlyIf(()-> !StateHandler.getInstance().getBBOneCovered()),
      AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToShoot"), constraints),
      new AutoScoreCommandGroup(swerve),

      new ParallelDeadlineGroup(
          AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ShootTo4"), constraints),
          new AutoIntake()
      ),
      new ParallelDeadlineGroup(
        new NoteSearch(swerve, NoteSearch.Direction.LEFT),
        new AutoIntake()
      ).onlyIf(()-> !StateHandler.getInstance().getBBOneCovered()),      
      AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToShoot"), constraints),
      new AutoScoreCommandGroup(swerve)
    );
  }
}
