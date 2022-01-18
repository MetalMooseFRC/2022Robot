// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.controller.PIDController;

public class VisionFollow extends CommandBase {
  Limelight limelight;
  DriveTrain driveTrain;

  private static final double kP = 7.0;
 // integral speed constant
  private static final double kI = 0.018;
  // derivative speed constant
  private static final double kD = 1.5;

  private final PIDController m_pidController = new PIDController(kP, kI, kD);

  /** Creates a new VisionFollow. */
  public VisionFollow(DriveTrain driveTrain, Limelight limelight) {
    addRequirements(driveTrain, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutput = m_pidController.calculate(limelight.getTx());

    driveTrain.drive.arcadeDrive(0.0, pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
