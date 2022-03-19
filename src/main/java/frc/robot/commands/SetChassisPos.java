package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Input_Mode;
import raiderlib.geometry.Pose;

public class SetChassisPos extends CommandBase {
  /**
   * Creates a new SetChassisAngle.
   */
  private final Chassis m_chassis;
  Pose pose;
  public SetChassisPos(Chassis chassis, Pose pose) {
    this.pose = pose;
    m_chassis = chassis;
    addRequirements(m_chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.setPose(pose.x, pose.y, pose.theta);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.setPose(pose.x, pose.y, pose.theta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeed(Input_Mode.PercentOut, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}