// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Elevator extends SubsystemBase {
  private final CANSparkMax m_motorLeft = new CANSparkMax(Constants.MOTOR_ELEVATOR_LEFT_4_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final RelativeEncoder leftElevatorEncoder = m_motorLeft.getEncoder();
  
  private final CANSparkMax m_motorRight = new CANSparkMax(Constants.MOTOR_ELEVATOR_RIGHT_4_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final RelativeEncoder rightElevatorEncoder = m_motorRight.getEncoder();

  public final MotorControllerGroup elevatorControllerGroup = new MotorControllerGroup(m_motorLeft, m_motorRight);
   
  
  /** Creates a new Elevator. */
  public Elevator() {
    m_motorRight.setInverted(true);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
