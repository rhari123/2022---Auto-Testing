// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TempClimber;



public class DefaultTempClimber extends CommandBase {
  /** Creates a new TempIntake. */
  private final TempClimber m_climber;
  private final XboxController m_XboxController;

  private double leftCount;
  private double rightCount;
  
  public DefaultTempClimber(TempClimber climber, XboxController xboxController) {
    m_climber = climber;
    m_XboxController = xboxController;
    addRequirements(m_climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    leftCount = m_climber.getLeftCt();
    rightCount = m_climber.getRightCt();
    // System.out.println("LCt: " + leftCount);
    // System.out.println("RCt: " + rightCount);
    // System.out.println("Bump: " + m_XboxController.getRawButton());
   if (m_XboxController.getRightBumperPressed() && leftCount <= 110000 && rightCount<= 110000){
      m_climber.up();
    }

    else if (m_XboxController.getRightBumperReleased()){
      m_climber.stop();
  }
 
    if (m_XboxController.getLeftBumperPressed() && leftCount >= 0 && rightCount >= 0){
      m_climber.down();
    }
    else if (m_XboxController.getLeftBumperReleased()){
      m_climber.stop();
    }


    if (m_XboxController.getBButtonPressed()){
      m_climber.setPosition(0);
    }

    // if (m_XboxController.getAButton()){
    //   m_climber.robotUp();
    // }
    // else if (m_XboxController.getAButtonReleased()){
    //   m_climber.stop();
    // }
    
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
