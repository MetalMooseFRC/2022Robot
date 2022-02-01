// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Turret extends SubsystemBase {

  // Left motor, encoder
  public final CANSparkMax motorLeft = new CANSparkMax(Constants.MOTOR_TURRET_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final RelativeEncoder leftTurretEncoder = motorLeft.getEncoder();
  // public final MotorControllerGroup leftTurretControllerGroup = new MotorControllerGroup(motorLeft);
  
  // Right motor, encoder
  public final CANSparkMax motorRight = new CANSparkMax(Constants.MOTOR_TURRET_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final RelativeEncoder rightTurretEncoder = motorRight.getEncoder();   
  // public final MotorControllerGroup rightTurretControllerGroup = new MotorControllerGroup(motorRight);

  /** Creates a new Turret. */
  public Turret() {
    motorRight.setInverted(true);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
