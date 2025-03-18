/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;

public class BlingSubsystem extends SubsystemBase {

  private CANdle candle;
  public final double Brightness = 0.5;
  private static BlingSubsystem INSTANCE = null;

  /**
   * Creates a new Bling. */
   
  private BlingSubsystem() {
    if (Config.BlingConstants.CANDLE != -1) {
      SubsystemChecker.subsystemConstructed(SubsystemType.BlingSubsystem);
      candle = new CANdle(Config.BlingConstants.CANDLE);

      CANdleConfiguration config = new CANdleConfiguration();
      config.stripType = LEDStripType.RGB; // set the strip type to RGB
      config.brightnessScalar = Brightness; // dim the LEDs to half brightness

      candle.configAllSettings(config);
    } else {
      candle = null;
    }
  }

  public static BlingSubsystem getINSTANCE() {
    if (Config.BlingConstants.CANDLE == -1) {
      INSTANCE = null;
    } else if (INSTANCE == null) {
      INSTANCE = new BlingSubsystem();
    }

    return INSTANCE;
  }


  public void setBrightness() {
    candle.configBrightnessScalar(Brightness);
  }

  public void setDisabled() {
    candle.configBrightnessScalar(0.0);
    candle.clearAnimation(0);
  }

  public void setOrange() {
    candle.clearAnimation(0);
    candle.setLEDs(245, 141, 66);
  }

  public void setPurple() {
    candle.clearAnimation(0);
    candle.setLEDs(138, 43, 226);
  }

  public void setBlue() {
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 255);
  }

  public void setRed() {
    candle.clearAnimation(0);
    candle.setLEDs(255, 0, 0);
  }

  public void setHoneydew() {
    candle.clearAnimation(0);
    candle.setLEDs(240, 255, 240);
  }

  public void setYellow() {
    candle.clearAnimation(0);
    candle.setLEDs(255, 255, 0);
  }

  // coral align
  public void setCoralAlign() {
    candle.clearAnimation(0);
    for (int i = 0; i < 512; i += 6) {
      if (i < 6) {
        candle.setLEDs(255, 0, 0, 0, i, 6); // Red (0-5)
      } else if (i < 12) {
        candle.setLEDs(255, 165, 0, 0, i, 6); // Orange (6-11)
      } else if (i < 18) {
        candle.setLEDs(255, 255, 0, 0, i, 6); // Yellow (12-17)
      } else if (i < 24) {
        candle.setLEDs(0, 255, 0, 0, i, 6); // Green (18-23)
      } else if (i < 30) {
        candle.setLEDs(0, 0, 255, 0, i, 6); // Blue (24-29)
      } else if (i < 36) {
        candle.setLEDs(75, 0, 130, 0, i, 6); // Indigo (30-35)
      } else if (i < 42) {
        candle.setLEDs(148, 0, 211, 0, i, 6); // Violet (36-41)
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setAnimation(Animation animation) {

    candle.animate(animation);
  }

} 