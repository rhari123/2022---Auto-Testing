// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Input_Mode;

public class GoBackFor5 extends CommandBase {
  /** Creates a new GoBackFor5. */
  private Chassis m_chassis;
  double startTime;
  public GoBackFor5(Chassis chassis) {
    m_chassis = chassis;
    addRequirements(m_chassis);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    //m_chassis.shiftToLow();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Timer.getFPGATimestamp() - startTime <= Constants.ShooterC.stallTime)
    {
      m_chassis.setSpeed(Input_Mode.PercentOut, -0.3, -0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime >= Constants.ShooterC.stallTime;
  }
}
