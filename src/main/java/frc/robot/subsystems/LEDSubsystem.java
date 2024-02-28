// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;

public class LEDSubsystem extends SubsystemBase {

  private int ledCount = 40;
  private RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, ledCount);

  private CANdle candle = new CANdle(CANdleConstants.candleID, "rio");

  public enum Colors {
    RED(new Color(255, 0, 0)),
    ORANGE(new Color(255, 165, 0)),
    WHITE(new Color(255, 255, 255)),
    PURPLE(new Color(160, 32, 240)),
    BLUE(new Color(0,0,255)),
    GREEN(new Color(0, 255,0)), 
    PINK(new Color(255, 192, 203)),
    YELLOW(new Color(255, 255, 0));

    Color rgb;

    private Colors(Color c){
        rgb = c;
    }

    public Color getRGB(){
      return rgb;
    }

  }


  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.stripType = LEDStripType.GRB;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll, 50);
    
  }

  /*
   * Animation = FLASH
   */
  public void setColor(Colors color, boolean wantAnimation){
    switch(color){
      case RED:
        if(wantAnimation){
          candle.animate(new StrobeAnimation(255, 0, 0, 0, 0.5, ledCount));
        }
        else{
          candle.setLEDs(255, 0, 0);
        }
      case ORANGE:
        if(wantAnimation){
          candle.animate(new StrobeAnimation(255, 165, 0, 0, 0.5, ledCount));
        }
        else{
          candle.setLEDs(255, 165, 0);
        }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /*
     * CASES TO ADD: 
     * INTAKE Deployment
     * INTAKING GAMEPIECE
     * AMP MODE
     * AMP MODE + SCORE
     * SUBWOOFER
     * SUBWOOFER + SCORE
     * BACKWARDS SUBWOOFER
     * BACKWARDS SUBWOOFER + SCORE
     * RANGED SHOT
     * RANGED SHOT + IN RANGE
     * RANGED SHOT + IN RANGE + SCORE
     * TRAP 
     * TRAP + TAG
     * TRAP + TAG + SCORE
     * CLIMB
     * SERVO
     */
  }
}
