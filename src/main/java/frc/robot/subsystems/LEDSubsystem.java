// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.lib.StateMachine.StateHandler;
import frc.robot.lib.StateMachine.StateVariables.ArmStates;
import frc.robot.lib.StateMachine.StateVariables.FeederSpeeds;
import frc.robot.lib.StateMachine.StateVariables.IntakeStates;
import frc.robot.lib.StateMachine.StateVariables.ShooterSpeeds;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new CANdleSubsystem. */

  StateHandler stateHandler = StateHandler.getInstance();

  private int LEDCount = Constants.LEDConstants.LEDCount;
  Timer flashTimer = new Timer();

  private Colors currentColor = Colors.OFF;

  private Animations currentAnimation = Animations.OFF;

  private CANdle candle = new CANdle(Constants.LEDConstants.CANdleID);

  public enum Colors {
    RED(new int[] { 255, 0, 0 }),
    GREEN(new int[] { 0, 255, 0 }),
    BLUE(new int[] { 0, 70, 255 }),
    PURPLE(new int[] { 90, 0, 255 }),
    PINK(new int[] { 255, 40, 200 }),
    ORANGE(new int[] { 255, 28, 0 }),
    YELLOW(new int[] { 255, 120, 0 }),
    WHITE(new int[] { 255, 255, 255 }),
    RAINBOW(new int[] { 0, 0, 0 }),
    OFF(new int[] { 0, 0, 0 }),
    TEST(new int[] { 0, 0, 0 });

    public int[] RGB;

    private Colors(int[] c) {
      RGB = c;
    }
  }

  public enum Animations {
    HEARTBEAT,
    OFF,
    SOLID,
    FLASHING,
    FIRE,
    LARSON,
    RAINBOW;

    public Animation anim;

  }

  public LEDSubsystem() {

    CANdleConfiguration config = new CANdleConfiguration();

    config.stripType = LEDStripType.GRB;
    config.v5Enabled = true;
    config.brightnessScalar = 1;
    candle.configAllSettings(config);

    // waitTimer.start();
  }

  public void apply(Colors c, Animations a) {
    int r = c.RGB[0];
    int g = c.RGB[1];
    int b = c.RGB[2];

    endAnimation();
    candle.clearAnimation(0);

    // trying useing animation slot 0 alwasy, maybe overrunning?

    switch (a) {
      case SOLID:
        candle.setLEDs(r, g, b, 0, 0, LEDCount);
        // candle.animate(new SingleFadeAnimation(r,g , b,0,0, LEDCount),0);
        break;
      case RAINBOW:
        candle.animate(new RainbowAnimation(), 0);
        break;
      case HEARTBEAT:
        candle.animate(new SingleFadeAnimation(r, g, b, 0, 0.7, LEDCount), 0);
        break;
      case FLASHING:
        candle.animate(new StrobeAnimation(r, g, b, 0, 0.3, LEDCount, 0), 0);
        break;
      case FIRE:
        candle.animate(new FireAnimation(1, 0.3, -1, 1, 1, false, 0));
        break;
      case LARSON:
        candle.animate(new LarsonAnimation(r, g, b, 0, 0.5, LEDCount, BounceMode.Front, 4), 0);
        break;
      default:
        candle.animate(new SingleFadeAnimation(0, 0, 0), 0);
        break;
    }

  }

  public void endAnimation() {
    candle.animate(null);
  }

  public void off() {
    candle.setLEDs(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Colors desiredColor = null;
    Animations desiredAnimation = null;

    boolean hasNote = stateHandler.getBBThreeCovered();

    // No Code

    // Has Code
    // Button Coast
    // button Zero

    // No Code/Comms

    // intake related'
    if (flashTimer.hasElapsed(2)) {
      flashTimer.stop();
      flashTimer.reset();
    }

    // regular
    if ((flashTimer.get() < 2) && ((stateHandler.getBBOneCovered() && !stateHandler.getBB1Dead())
        || (stateHandler.getDesiredArmState() == ArmStates.BABY_BIRD && stateHandler.getBBFourCovered()))) { // baby
                                                                                                             // bird

      desiredColor = Colors.WHITE;
      desiredAnimation = Animations.FLASHING;

      if (flashTimer.get() == 0) {
        flashTimer.start();
      }
    } else if (stateHandler.getManuallyClimbing()) {
      desiredColor = Colors.PINK;
      desiredAnimation = Animations.HEARTBEAT;
    } else if (stateHandler.getCurrentIntakeState() == IntakeStates.DEPLOYED) { // doesn't always go
      desiredColor = Colors.ORANGE;
      desiredAnimation = Animations.SOLID;
    }

    else if (hasNote) {

      // for amp
      if (stateHandler.getScoreInAmp()) {
        desiredColor = Colors.ORANGE;
        if (stateHandler.getCurrentFeederSpeed() == FeederSpeeds.OUTWARD) {
          desiredAnimation = Animations.SOLID;
        } else {
          desiredAnimation = Animations.HEARTBEAT;
        }
      }

      // for subwoofer
      else if (stateHandler.getScoreInSubwoofer()) {

        desiredColor = Colors.BLUE;
        if (stateHandler.getCurrentFeederSpeed() == FeederSpeeds.INWARD
            && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT) {
          desiredAnimation = Animations.SOLID;
        } else {
          desiredAnimation = Animations.HEARTBEAT;
        }
      }
      // for reverse subwoofer
      else if (stateHandler.getScoreInReverseSubwoofer()) {

        desiredColor = Colors.PURPLE;
        if (stateHandler.getCurrentFeederSpeed() == FeederSpeeds.INWARD
            && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT) {
          desiredAnimation = Animations.SOLID;
        } else {
          desiredAnimation = Animations.HEARTBEAT;
        }

      }
      // for trap
      else if (stateHandler.getScoreInTrap()) {
        desiredColor = Colors.OFF;

        if (stateHandler.getHasValidTrapTag()) {
          if (stateHandler.getCurrentFeederSpeed() == FeederSpeeds.INWARD
              && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT) {
            desiredAnimation = Animations.RAINBOW;
            desiredColor = Colors.RAINBOW;
          } else {
            desiredAnimation = Animations.SOLID;
          }
        } else {
          desiredAnimation = Animations.HEARTBEAT;
        }
      }
      // implied ranged
      else {
        desiredColor = Colors.GREEN;
        if (stateHandler.getHasValidSpeakerTag()
            && (stateHandler.getDistanceToSpeakerTag() < LimeLightConstants.speakerLerpUpperBound
                && stateHandler.getDistanceToSpeakerTag() > LimeLightConstants.speakerLerpLowerBound)) {
          if (stateHandler.getCurrentFeederSpeed() == FeederSpeeds.INWARD
              && stateHandler.getCurrentShootingSpeed() == ShooterSpeeds.SHOOT) {
            desiredColor = Colors.RAINBOW;
            desiredAnimation = Animations.RAINBOW;
          } else {
            desiredAnimation = Animations.RAINBOW;
            desiredColor = Colors.RAINBOW;
          }
        } else {
          desiredAnimation = Animations.HEARTBEAT;
        }

      }
    } else {
      desiredColor = Colors.RED;
      desiredAnimation = Animations.FIRE;
    }

    SmartDashboard.putString("CurrentAnim", currentAnimation.name());
    SmartDashboard.putString("CurrentColor", currentColor.name());

    SmartDashboard.putString("desiredAnim", desiredAnimation.name());
    SmartDashboard.putString("desiredColor", desiredColor.name());

    // SmartDashboard.putNumber("WaitTimer", waitTimer.get());

    if (/* waitTimer.hasElapsed(0.5) && */ (desiredColor != null && desiredAnimation != null)
        && (currentAnimation != desiredAnimation || currentColor != desiredColor)) {
      apply(desiredColor, desiredAnimation);
      // System.out.println("Applied " +desiredColor.name()+ "
      // "+desiredAnimation.name());
      currentColor = desiredColor;
      currentAnimation = desiredAnimation;
      // waitTimer.restart();
    }
  }
}
