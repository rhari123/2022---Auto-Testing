// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CameraServo;

public class DefaultServo extends CommandBase {
  /**
   * Creates a new DefaultServo.
   */
  CameraServo mServoMotor;
  Joystick m_leftstick = new Joystick(Constants.Joystick.leftStick);

  public DefaultServo(CameraServo servoMotor, Joystick leftstick) {
    // Use addRequirements() here to declare subsystem dependencies.
    mServoMotor = servoMotor;
    m_leftstick = leftstick;
    addRequirements(mServoMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mServoMotor.setAngle(0);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}