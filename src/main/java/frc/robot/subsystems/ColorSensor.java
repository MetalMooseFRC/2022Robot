// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor extends SubsystemBase {
  

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  

  /** Creates a new ColorSensor. */
  public ColorSensor() {}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = m_colorSensor.getColor();

    // red value detected
    final double red = detectedColor.red;
    // blue value detected
    final double blue = detectedColor.blue;

    // If color is greater than other by COLOR_DIFFERENCE, then put on screen
    if (red - blue > Constants.COLOR_DIFFERENCE){
      SmartDashboard.putString("color", "Red");
    }
    if (blue - red > Constants.COLOR_DIFFERENCE){
      SmartDashboard.putString("color", "Blue");
    }
    // otherwise put unknown
    else {
      SmartDashboard.putString("color", "Uknown?");
    }

    // Put all colors for debugging
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);

  }
}
