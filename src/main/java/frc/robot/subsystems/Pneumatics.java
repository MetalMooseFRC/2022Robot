// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  // From https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html
  // What does this mean??? 
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /** Creates a new Pneumatics. */
  public Pneumatics() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
