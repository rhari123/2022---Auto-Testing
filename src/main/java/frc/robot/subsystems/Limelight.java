// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  NetworkTableEntry tx; //horizontal offset
  NetworkTableEntry ty; //vertical offset
  NetworkTableEntry tv; //valid target
  NetworkTableEntry tl;
  NetworkTable table;
  
  double distance;
  double y_theta;
  double x_theta;

  boolean zoomed;
  boolean valid;
  double pipeLatency;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("ledMode").setNumber(3);
    table.getEntry("pipeline").setNumber(0);
    table.getEntry("canMode").setNumber(0);
    zoomed = false;
  }

  public double getTX(){
    return ty.getDouble(0);
  }

  public void update()
  {
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    tl = table.getEntry("tl");
    pipeLatency = tl.getDouble(0.0) / 1000;
    valid = (tv.getDouble(0.0) > 0) ? true : false;
    y_theta = ty.getDouble(0) + Constants.Limelight.kLimelightAngle; 
    // distance = (Constants.Limelight.kTargetHeight - Constants.Limelight.kLimeLightHeight) / Math.tan(y_theta);
    distance = (Constants.Limelight.kTargetHeight - Constants.Limelight.kLimeLightHeight) / Math.tan(y_theta *(Math.PI/180));
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean ("target", valid);
    // SmartDashboard.putNumber("LimelightY", y_theta);
    SmartDashboard.putNumber("Distance", distance);
  }
}
