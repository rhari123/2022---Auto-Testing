// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Tower;

public class DefaultTower extends CommandBase {

  private final Tower m_tower;
  private final XboxController m_xboxController;

  boolean occupied;

  /** Creates a new AutoIndexer. */
  public DefaultTower(Tower indexer, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tower = indexer;
    addRequirements(m_tower);
    m_xboxController = xboxController;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // occupied = m_tower.isOccupied();

    if (m_tower.getEnt()) {
      if (!occupied) {
        m_tower.towerStop();
      } else
        // Theoretically this line will never run, becuase ball cannot possibly hit the
        // entrance switch when another one is stopped at the bottom, just as a
        // precation is placed in case it glitches
        m_tower.towerStop();
    } else {
      m_tower.towerIdle();
    }

    // m_tower.towerStop();

    // Sends ball up the tower, shooter is already running and at speed
    if (m_xboxController.getRightTriggerAxis() > 0.5 ) {
      m_tower.towerUp();
    }
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
