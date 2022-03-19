// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Input_Mode;

public class JoystickDriving extends CommandBase {
  /** Creates a new JoystickDriving. */

  private final Chassis m_chassis;
  private final Joystick m_leftStick;
  private final Joystick m_rightStick;
  
  public JoystickDriving(Chassis chassis, Joystick leftStick, Joystick rightStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_chassis = chassis;
    m_leftStick = leftStick; 
    m_rightStick = rightStick;
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(-m_leftStick.getY() + Math.sin(m_rightStick.getX() * Math.PI/2));
    System.out.println(-m_leftStick.getY() - Math.sin(m_rightStick.getX() * Math.PI/2));
    m_chassis.setSpeed(Input_Mode.PercentOut, -m_leftStick.getY() - Math.sin(m_rightStick.getX() * Math.PI/2), -m_leftStick.getY() + Math.sin(m_rightStick.getX() * Math.PI/2));
    // m_chassis.setSpeed(Input_Mode.PercentOut, m_leftStick.getY(), m_rightStick.getY());
    // m_chassis.setSpeed(Input_Mode.PercentOut, -m_leftStick.getY(), -m_rightStick.getY());
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
