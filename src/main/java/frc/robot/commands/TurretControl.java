// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretControl extends CommandBase {

  private Turret m_turret;
  private DoubleSupplier m_angle;

  // TODO: Need to make this constants and tune properly
  private final PIDController m_leftpidController = new PIDController(0.2, 0, 0);
  private final PIDController m_rightpidController = new PIDController(0.2, 0, 0);


  /** Creates a new turretControl. */
  public TurretControl(Turret turret, DoubleSupplier angle) {
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset position
    m_turret.leftTurretEncoder.setPosition(0);

    // TODO: Tune / make constant
    m_leftpidController.setTolerance(0.2);
    m_rightpidController.setTolerance(0.2);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_leftpidController.setSetpoint(0);
    m_rightpidController.setSetpoint(0);

    double LeftpidOutput = m_leftpidController.calculate(m_turret.leftTurretEncoder.getPosition());
    double RightpidOutput = m_rightpidController.calculate(m_turret.rightTurretEncoder.getPosition());

    SmartDashboard.putNumber("LeftpidOutput Turret", LeftpidOutput);
    SmartDashboard.putNumber("RightpidOutput Turret", RightpidOutput);

    
    //if (pidOutput < 0){pidOutput -= Constants.VISION_FF;}
    //else if (pidOutput > 0){pidOutput += Constants.VISION_FF;}
    // Make sure PID Controller is capped to 1 (more would be bad)
    double clamp = .1;
    LeftpidOutput = Math.max(-clamp, Math.min(clamp, LeftpidOutput));
    RightpidOutput = Math.max(-clamp, Math.min(clamp, RightpidOutput));

    m_turret.rightTurretEncoder.set(LeftpidOutput);
    m_turret.leftTurretEncoder.set(RightpidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.leftTurretEncoder.set(0);
    m_turret.rightTurretEncoder.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_pidController.atSetpoint()){return true;}
    return false;
  }
}
