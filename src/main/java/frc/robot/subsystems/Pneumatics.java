// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Using "import static an.enum.or.constants.inner.class.*;" helps reduce verbosity
// this replaces "DoubleSolenoid.Value.kForward" with just kForward
// further reading is available at https://www.geeksforgeeks.org/static-import-java/
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class Pneumatics extends SubsystemBase {
  // From https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  public final DoubleSolenoid exampleDoublePH = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, 4, 5);
  /** Creates a new Pneumatics. */
  public Pneumatics() {
    exampleDoublePH.set(kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
