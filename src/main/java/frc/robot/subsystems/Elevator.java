// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//includes a group of two SmartMax MC'c (turning in opposite directions)
//Add a command (no default for the Elevator) to run the motors from the operator's joystick: stick forward = motion one way; stick backward = motion other way
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Elevator {
    private final CANSparkMax m_motorLeft4 = new CANSparkMax(Constants.MOTOR_LEFT_1_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  
    private final CANSparkMax m_motorRight4 = new CANSparkMax(Constants.MOTOR_RIGHT_1_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    /** Creates a new DriveTrain. */
    public ElevatorDrive() {
      // create deadband
      drive.setDeadband(Constants.DEADBAND);
      // invert right motors
      m_motorRight4.setInverted(true);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
