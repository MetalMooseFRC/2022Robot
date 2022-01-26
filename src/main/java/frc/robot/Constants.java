// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
	
	public static final int MOTOR_ELEVATOR_LEFT_4_ID = 7;
	public static final int MOTOR_ELEVATOR_RIGHT_4_ID = 8;

	public static final int DRIVER_STICK_PORT = 0;
	public static final int OP_STICK_PORT = 1;
	
	public static final double MIN_JOYSTICK_INPUT = 0.1;
	// Must be <= 1
	public static final double JOYSTICK_SPEED_FACTOR = 1;
	// Must be <= 1
	public static final double JOYSTICK_TURN_FACTOR = 0.85;

	public static final double ELEVATOR_SPEED_FACTOR = 0.5;
	
	
	public static final double TARGET_HEIGHT = 57;  //inches
	public static final double LIMELIGHT_HEIGHT = 39.25; //inches

	// Decrease these numbers to make more sensitive
	public static final double BLUE_SENSITIVITY = 0.15;
	public static final double RED_SENSITIVITY = 0.30;

	public static final double DEADBAND = 0.07;

	//Proportional [P] Set point p means motor speed is going to be proportional to the amount of error, closer you get less motor speed
	//Integral [I] it adds up the errors over time, final adjustments
	//Derivative speed constant [D]
	//Feed Forward [F] constant value applied to motors at all times, like zeroing scale, elevator needs to hold its weight
	public static final double VISION_KP = 0.85;
	// integral speed constant
  	public static final double VISION_KI = 0.0;
	// derivative speed constant [D]
	public static final double VISION_KD = 0.05;
	// feed forward
	public static final double VISION_FF = 0.14;
	// robot can see 27*2 degrees and so this gives some room for error
	public static final double VISION_FOV_ERROR = 5/27;
}
