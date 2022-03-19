// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class DefaultIntake extends CommandBase {
  private final Intake m_intake;
  private final XboxController m_xboxController;
  /** Creates a new DefaultIntake. */
  public DefaultIntake(Intake intake, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake; 
    m_xboxController = xboxController;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_intake.up();
    // m_intake.stop();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_xboxController.getLeftTriggerAxis() > 0.5) {
      m_intake.down();
      m_intake.takeIn();
    }

    else {
      m_intake.stop();
      m_intake.up();
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
