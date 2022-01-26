// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorControl extends CommandBase {

  private Elevator m_elevator;

  private DoubleSupplier m_speedSupplier;

  /** Creates a new ElevatorControl. */
  public ElevatorControl(Elevator elevator, DoubleSupplier speedSupplier) {
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_speedSupplier = speedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevatorUpDown = Constants.ELEVATOR_SPEED_FACTOR * m_speedSupplier.getAsDouble();
    m_elevator.elevatorControllerGroup.set(elevatorUpDown);
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
