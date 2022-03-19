// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Input_Mode;

public class FollowPath extends CommandBase {
  /** Creates a new FollowPath. */
  private final Chassis m_chassis;
  private final RamseteController m_ramseteController;
  private Trajectory trajectory;
  private final String trajectoryJSON;
  double st;

  public FollowPath(Chassis chassis, String tj) {
    m_chassis = chassis;
    m_ramseteController = new RamseteController(Constants.Autos.kRamseteB, Constants.Autos.kRamseteZeta);
    trajectoryJSON = tj;
    addRequirements(m_chassis);
    // Use addRequirements() here to declare subsystem dependencies.
    try {
        // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Path path = FileSystems.getDefault().getPath("PathWeaver/output", "PickUpBall.wpilib.json");
        trajectory = TrajectoryUtil.fromPathweaverJson(path);
        System.out.println("Caught Path");
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    st = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = trajectory.sample(Timer.getFPGATimestamp() - st);
    ChassisSpeeds adjSpeeds = m_ramseteController.calculate(new Pose2d(m_chassis.getCurrentPose().x, m_chassis.getCurrentPose().y, new Rotation2d(m_chassis.getCurrentPose().theta)), goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = (new DifferentialDriveKinematics(10)).toWheelSpeeds(adjSpeeds);
    m_chassis.setSpeed(Input_Mode.Inches, wheelSpeeds.leftMetersPerSecond * 39.3701, wheelSpeeds.rightMetersPerSecond * 39.3701);
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
