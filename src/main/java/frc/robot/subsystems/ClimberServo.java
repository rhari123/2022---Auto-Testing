// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberServo extends SubsystemBase {
  /** Creates a new ClimberServo. */
  Double angle = 0.0;
  Servo leftClimber;
  Servo rightClimber; 
  Boolean clamped = false;
  public ClimberServo() {
    leftClimber = new Servo(1);
    rightClimber = new Servo(2);
  }
  
  public void setAngle(double degrees) {
    double setVal = degrees / 180;
    angle = degrees;
    rightClimber.set(setVal);
    leftClimber.set(setVal);
  }
  
  public void clamp(){
    setAngle(45);
    clamped = true;
  }

  public void free()
  {
    setAngle(180);
    clamped = false;
  }

  public boolean isClamped()
  {
    return clamped;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
