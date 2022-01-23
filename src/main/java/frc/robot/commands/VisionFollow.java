// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import java.lang.Math;
import java.security.ProviderException;

public class VisionFollow extends CommandBase {
  Limelight m_limelight;
  DriveTrain m_driveTrain;

  private static final double kP = Constants.VISION_KP;
 // integral speed constant
  private static final double kI = Constants.VISION_KI;
  // derivative speed constant
  private static final double kD = Constants.VISION_KD;

  private final PIDController m_pidController = new PIDController(kP, kI, kD);

  /** Creates a new VisionFollow. */
  public VisionFollow(Limelight limelight, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, driveTrain);
    m_limelight = limelight;
    m_driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setSetpoint(0.0);
    m_pidController.setTolerance(Constants.VISION_FOV_ERROR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutput = m_pidController.calculate(m_limelight.x / 27);
    
    if (pidOutput < 0){pidOutput -= Constants.VISION_FF;}
    else if (pidOutput > 0){pidOutput += Constants.VISION_FF;}
    // Make sure PID Controller is capped to 1 (more would be bad)
    double clamp = 1;
    pidOutput = Math.max(-clamp, Math.min(clamp, pidOutput));

    SmartDashboard.putNumber("PID Output", pidOutput);
    m_driveTrain.drive.arcadeDrive(0.0, -pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
