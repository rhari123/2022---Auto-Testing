// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;

public class ReverseIntake extends CommandBase {
  /** Creates a new ReverseIntake. */
  Intake m_intake;
  Tower m_tower;
  Joystick m_leftStick; 
  public ReverseIntake(Intake intake, Tower tower, Joystick leftStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_tower = tower;
    m_leftStick = leftStick;
    addRequirements(m_intake, m_tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tower.towerDown(); //Sends the ball down the tower
    m_intake.down(); //Puts the intake arm down(pistons)
    m_intake.ballOut();//Reverses the speed of the intake, sends balls out
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.up();
    m_intake.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_leftStick.getRawButtonReleased(Constants.Joystick.intakeOutButton);
  }
}
