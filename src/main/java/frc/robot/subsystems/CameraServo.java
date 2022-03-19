// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraServo extends SubsystemBase {
  Servo sMotor;
  Double angle = 0.0;

  public CameraServo() {
    sMotor = new Servo(0);
    zero();
  }

  public void zero() {
    sMotor.set(0.0);
  }

  public void setAngle(double degrees) {
    double setVal = degrees / 180;
    angle = degrees;
    sMotor.set(setVal);
  }

  public boolean isLow() {
    if (angle == Constants.ServoC.lowAngle){
      return true;
    }
    else
    return false;
  }

  public boolean isMedium() {
    if (angle == Constants.ServoC.mediumAngle){
      return true;
    }
    else
    return false;
  }
  public boolean isHigh() {
    if (angle == Constants.ServoC.highAngle){
      return true;
    }
    else
    return false;
  }

  // public boolean isZero() {
  //   if (angle == 0){
  //     return true;
  //   }
  //   else
  //   return false;
  // }

  public double getAngle(){
    return angle; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Servo Angle", angle);
  }
}
