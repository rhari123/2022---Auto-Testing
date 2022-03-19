/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Input_Mode;
import raiderlib.control.DriveCommand;
import raiderlib.control.PurePursuitController;
import raiderlib.path.Path;

public class PursuitPath extends CommandBase {
  /**
   * Creates a new PursuitPath.
   */
  private final Chassis m_chassis;
  private final Path m_path;
  private final PurePursuitController m_purePursuitController;
  private DriveCommand m_driveCommand;

  public PursuitPath(Chassis chassis, Path path, double minLD, double maxLD) {
    m_chassis = chassis;
    m_path = path;
    // Use addRequirements() here to declare subsystem dependencies.
    m_purePursuitController = new PurePursuitController(m_path, m_chassis.getDriveCharacterization(), minLD, maxLD);
    addRequirements(m_chassis);
  }

  public PursuitPath(Chassis chassis, Path path, double maxVelocity, double maxAcceleration, double maxDeceleration,
      double minLD, double maxLD) {
    m_chassis = chassis;
    m_path = path;
    // Use addRequirements() here to declare subsystem dependencies.
    m_purePursuitController = new PurePursuitController(m_path, maxVelocity, maxAcceleration, maxDeceleration,
        chassis.getDriveCharacterization().trackWidth, minLD, maxLD);
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_purePursuitController.reset();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_driveCommand = m_purePursuitController.pursuit_path(m_chassis.getPose());
    m_chassis.setSpeed(Input_Mode.Inches, m_driveCommand.left_command, m_driveCommand.right_command);
    // System.out.println(m_driveCommand.left_command + "," + m_driveCommand.right_command);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("finished");
    m_purePursuitController.reset();
    m_chassis.setSpeed(Input_Mode.PercentOut, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_purePursuitController.is_finished();
  }
}