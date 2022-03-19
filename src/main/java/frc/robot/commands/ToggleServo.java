// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CameraServo;

public class ToggleServo extends CommandBase {
  /** Creates a new ToggleServo. */

  CameraServo m_servo;
  Joystick m_leftStick; 
  public ToggleServo(CameraServo servo, Joystick leftStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_servo = servo; 
    m_leftStick = leftStick;
    addRequirements(m_servo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_servo.setAngle(Constants.ServoC.mediumAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_leftStick.getRawButtonPressed(9)){
      if (m_servo.isLow()){
        m_servo.setAngle(Constants.ServoC.mediumAngle);
      }
  
      else if (m_servo.isMedium()){
        m_servo.setAngle(Constants.ServoC.highAngle);
      }
  
      else if (m_servo.isHigh()){
        m_servo.setAngle(Constants.ServoC.lowAngle);
      }
    }
   
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
