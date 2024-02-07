// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Swerve;


// import java.util.function.DoubleSupplier;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.lib.LimelightUtil.LimelightInterface;
// import frc.robot.Constants.Swerve;
// import frc.robot.subsystems.SwerveSubsystem;

// public class MoveToGamePieceCommand extends Command {
//   private final SwerveSubsystem s_Swerve;
//   private final LimelightInterface limelight = LimelightInterface.getInstance();

//   private SlewRateLimiter translateLimiter, strafeLimiter, rotateLimiter;

//   private final double kPTarget = 0.004; // Tune this by yourself.
//   private PIDController target;

//   /** Creates a new GoalCentricCommand. */
//   public MoveToGamePieceCommand(SwerveSubsystem s) {
//     this.s_Swerve = s;
    
//     this.translateLimiter = new SlewRateLimiter(Swerve.maxAccel);
//     this.strafeLimiter = new SlewRateLimiter(Swerve.maxAccel);
//     this.rotateLimiter = new SlewRateLimiter(Swerve.maxAngularAccel);
    

//     target =  new PIDController(kPTarget, 0, 0);

//     addRequirements(s_Swerve);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
  

//     double rotationVal = 0;

//     if (limelight.hasNote()) {
//       rotationVal = target.calculate(limelight.getNoteXAngleOffset(), 0);
//     } else {
//       rotationVal = 0;
//     }
    
//     /*
//      * This needs extra logic because as theta goes to 0, cos goes to max, which isn't nessesarily the behavior that we want. 
//      * Maybe have an else, if Cos is close enough to 0, start just moving forward at const speed until BB tells us we have note,
//      * or it doesn't see note anymore?
//      */

//     double translationVal = Math.cos(Math.toRadians(limelight.getXNoteAngleOffset())); //cos of limelight angle is proportional to forward (towards note)
//     double strafeVal = Math.sin(Math.toRadians(limelight.getXNoteAngleOffset())); //sin of limelight angle is proportional to side to side relative to note
//     // rotationVal = Math.abs(rotationVal) > 0.01 ? rotationVal : 0.0;

//     translationVal = translateLimiter.calculate(translationVal) * Swerve.maxSpeed;
//     strafeVal = strafeLimiter.calculate(strafeVal) * Swerve.maxSpeed;
//     rotationVal = MathUtil.applyDeadband(rotationVal,0.005) * Swerve.maxAngularVelocity;

//     SmartDashboard.putNumber("ROTATIONVAL", rotationVal);

//     ChassisSpeeds chassisSpeeds;

//     //CHANGED TO ROBOT RELATIVE, because don't need field relative. 
//     //Strafe may need to be opposite of what it is RN, since left is +
//     if (DriverStation.getAlliance().get() == Alliance.Blue) {
//       chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(translationVal, strafeVal, rotationVal, s_Swerve.getGyroYaw());
//     } else {
//       chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(-translationVal, -strafeVal, rotationVal, s_Swerve.getGyroYaw());
//     }

//     SwerveModuleState[] moduleStates = Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
//     s_Swerve.setModuleStates(moduleStates);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return !limelight.hasNote() || robot has note;
//   }
// }
