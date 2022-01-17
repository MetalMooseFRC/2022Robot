// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  
  /** Creates a new Limelight. */
  public Limelight() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  //get the x error between the crosshair and target
  public double getTx() {
      return table.getEntry("tx").getDouble(0.0);
  }

  //get the y error between the crosshair and target
  public double getTy() {
    return table.getEntry("ty").getDouble(0.0); 
  }

  //get the area of the target
  public double getTa() {
    return table.getEntry("ta").getDouble(0.0);
}


  //does the limelight see a viable target
  public boolean hasValidTarget() {
    return table.getEntry("tv").getDouble(0.0) == 1;
  }

  //calculate the distance based on trig
  public double getDistance() {
      return (Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT)/Math.tan(getTy()*Math.PI/180);
  }

}
