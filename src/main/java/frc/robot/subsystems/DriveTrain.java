// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax m_motorLeft1 = new CANSparkMax(Constants.MOTOR_LEFT_1_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorLeft2 = new CANSparkMax(Constants.MOTOR_LEFT_2_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorLeft3 = new CANSparkMax(Constants.MOTOR_LEFT_3_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final MotorControllerGroup m_motorsLeft = new MotorControllerGroup(m_motorLeft1, m_motorLeft2, m_motorLeft3);

  private final CANSparkMax m_motorRight1 = new CANSparkMax(Constants.MOTOR_RIGHT_1_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorRight2 = new CANSparkMax(Constants.MOTOR_RIGHT_2_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_motorRight3 = new CANSparkMax(Constants.MOTOR_RIGHT_3_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final MotorControllerGroup m_motorsRight = new MotorControllerGroup(m_motorRight1, m_motorRight2, m_motorRight3);
  
  public final DifferentialDrive drive = new DifferentialDrive(m_motorsLeft, m_motorsRight);
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
