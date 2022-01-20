// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int MOTOR_LEFT_1_ID = 4;
	public static final int MOTOR_LEFT_2_ID = 5;
	public static final int MOTOR_LEFT_3_ID = 6;
    
    public static final int MOTOR_RIGHT_1_ID = 1;
    public static final int MOTOR_RIGHT_2_ID = 2;
	public static final int MOTOR_RIGHT_3_ID = 3;
	
	public static final int DRIVER_STICK_PORT = 0;
	
	public static final double MIN_JOYSTICK_INPUT = 0.1;
	public static final double JOYSTICK_SPEED_FACTOR = 1.2;
	public static final double JOYSTICK_TURN_FACTOR = 1.2;
	
	
	public static final double TARGET_HEIGHT = 57;  //inches
	public static final double LIMELIGHT_HEIGHT = 39.25; //inches

	// Decrease these numbers to make more sensitive
	public static final double BLUE_SENSITIVITY = 0.15;
	public static final double RED_SENSITIVITY = 0.30;

	public static final double DEADBAND = 0.07;

	public static final double VISION_KP = 0.5;
 	// integral speed constant
  	public static final double VISION_KI = 0.3;
  	// derivative speed constant
  	public static final double VISION_KD = 0.35;
}
