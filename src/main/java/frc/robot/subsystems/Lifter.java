// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lifter extends SubsystemBase {
  private ShuffleboardTab lifterTab = Shuffleboard.getTab("Lifter");
  NetworkTableEntry m_lifterSpeed = lifterTab.add("Lifter Speed", 0).getEntry();
  NetworkTableEntry m_lifterToggle = lifterTab.add("Lifter Toggle", 0).getEntry();
  public final CANSparkMax motor = new CANSparkMax(Constants.LIFTER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  /** Creates a new Lifter. */
  public Lifter() {
    
  }

  @Override
  public void periodic() {}

  public double getControlSpeed() {
    if (m_lifterToggle.getBoolean(false) == false){
      return 0.0;
    }
    else {
      return m_lifterSpeed.getDouble(0);
    }
  }
}

