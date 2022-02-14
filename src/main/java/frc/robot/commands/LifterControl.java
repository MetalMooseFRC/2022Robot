// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lifter;

public class LifterControl extends CommandBase {
  Lifter m_lifter;
  double m_speed;
  double slider;
  private ShuffleboardTab lifter = Shuffleboard.getTab("Lifter");
  /** Creates a new LifterControl. */
  public LifterControl(Lifter lifter, DoubleSupplier speed) {
    m_lifter = lifter;
    m_speed = speed.getAsDouble();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // FIXME: Otto do the smartdashboard --- toggle when 0 has motors off, toggle at 1 makes it go at slider speed
      m_lifter.motor.set(m_speed);
      // This method will be called once per scheduler run
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
