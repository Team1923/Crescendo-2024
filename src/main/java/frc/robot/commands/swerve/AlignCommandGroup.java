// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignCommandGroup extends SequentialCommandGroup {

  StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new AlignCommandGroup. */
  public AlignCommandGroup(CommandSwerveDrivetrain swerve, DoubleSupplier translation, DoubleSupplier strafe) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (stateHandler.getWantFrontAmp() || stateHandler.getScoreInAmp()){
      addCommands(new Align90(swerve, translation, strafe));
    }
    else if (stateHandler.getScoreInTrap()){
      addCommands(new LockHeadingToTrap(swerve, translation, strafe));
    }

    
  }
}
