// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ControllerConstants.Driver;
import frc.robot.lib.Controller.ControllerLimiter;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.SwerveRequests;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.lib.SwerveRequests.GoalCentricRequest;

public class ManageRequests extends Command {

  private CommandSwerveDrivetrain swerve;


  StateHandler stateHandler = StateHandler.getInstance();


  private DoubleSupplier translationSupplier;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier rotationSupplier;

  /** Creates a new ManageRequests. */
  public ManageRequests(CommandSwerveDrivetrain swerve, DoubleSupplier t, DoubleSupplier s, DoubleSupplier r) {
    // Use addRequirements() here to declare subsystem dependencies.


    this.swerve = swerve;
    this.translationSupplier = t;
    this.strafeSupplier = s;
    this.rotationSupplier = r;

    addRequirements(this.swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveRequests currentRequest = stateHandler.getCurrentSwerveRequest();

    //default
    SwerveRequest requestObj = SwerveRequests.FIELD_CENTRIC.request;

    double translation = translationSupplier.getAsDouble() * sideInversions()[0];
    double strafe = strafeSupplier.getAsDouble() * sideInversions()[1];
    double rotation = rotationSupplier.getAsDouble() * sideInversions()[2];

        if (currentRequest == SwerveRequests.AUTO && !DriverStation.isAutonomous()){
          System.out.println("SWERVE STILL THINKS IN AUTO");
          currentRequest = SwerveRequests.FIELD_CENTRIC;
        }
        
        if (currentRequest == SwerveRequests.FIELD_CENTRIC){

          requestObj = ((SwerveRequest.FieldCentric)SwerveRequests.FIELD_CENTRIC.request)
            .withVelocityX(translation * SwerveConstants.maxSpeed)
            .withVelocityY(strafe * SwerveConstants.maxSpeed)
            .withRotationalRate(rotation * SwerveConstants.maxAngularRate);
        

        }
        else if (currentRequest == SwerveRequests.GOAL_CENTRIC){

            requestObj = ((SwerveRequest.FieldCentricFacingAngle)SwerveRequests.GOAL_CENTRIC.request)
            .withVelocityX(translation * SwerveConstants.maxSpeed)
            .withVelocityY(strafe * SwerveConstants.maxSpeed)
            .withTargetDirection(Rotation2d.fromDegrees(swerve.getGyroYaw().getDegrees()-stateHandler.getxAngleOffset()));
  
            

           

        }
        else if (currentRequest == SwerveRequests.FACING_AMP){

          requestObj =((SwerveRequest.FieldCentricFacingAngle)SwerveRequests.FACING_AMP.request)
            .withVelocityX(translation * SwerveConstants.maxSpeed)
            .withVelocityY(strafe * SwerveConstants.maxSpeed)
            .withTargetDirection(Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : -90));
            
        }
        else if (currentRequest == SwerveRequests.NOTE_SEARCHING){
           requestObj =((SwerveRequest.FieldCentricFacingAngle)SwerveRequests.NOTE_SEARCHING.request)
            .withVelocityX(0.5 * sideInversions()[0] * SwerveConstants.maxSpeed)
            .withVelocityY(0.5 * sideInversions()[1] * SwerveConstants.maxSpeed)
            .withTargetDirection(Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : -90));

            System.out.println("HERER TOO");
        }

        else if (currentRequest == SwerveRequests.ROBOT_CENTRIC){
          requestObj =((SwerveRequest.RobotCentric)SwerveRequests.ROBOT_CENTRIC.request)
            .withVelocityX(translation * SwerveConstants.maxSpeed)
            .withVelocityY(strafe * SwerveConstants.maxSpeed)
            .withRotationalDeadband(ControllerConstants.Driver.deadband * SwerveConstants.maxAngularRate);
            
        }
        

        swerve.setControl(requestObj);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public static int[] sideInversions(){
    return (DriverStation.getAlliance().get() == Alliance.Blue) ? new int[]{1,1,1}
                                                                : new int[]{-1,-1, 1};
  }
}
