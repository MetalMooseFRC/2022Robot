// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorControl extends CommandBase {

  private Elevator m_elevator;
  private int m_button;

  // TODO: Need to make this constants and tune properly
  private final PIDController m_pidController = new PIDController(0.2, 0, 0);

  /** Creates a new ElevatorControl. */
  public ElevatorControl(Elevator elevator, int button) {
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_button = button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset position
    m_elevator.leftElevatorEncoder.setPosition(0);

    // Button switch case
    switch (m_button){
      case Constants.OP_ELEVATOR_UP_BUTTON: m_pidController.setSetpoint(Constants.ELEVATOR_RANGE); break;
      case Constants.OP_ELEVATOR_DOWN_BUTTON: m_pidController.setSetpoint(-Constants.ELEVATOR_RANGE); break;
    }
    // TODO: Tune / make constant
    m_pidController.setTolerance(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutput = m_pidController.calculate(m_elevator.leftElevatorEncoder.getPosition());
    SmartDashboard.putNumber("pidOutput Elevator", pidOutput);
    
    // Make sure PID Controller is capped to at most 1 (more would be bad)
    double clamp = .1;
    pidOutput = Math.max(-clamp, Math.min(clamp, pidOutput));

    m_elevator.elevatorControllerGroup.set(pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.elevatorControllerGroup.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_pidController.atSetpoint()){return true;}
    return false;
  }
}
