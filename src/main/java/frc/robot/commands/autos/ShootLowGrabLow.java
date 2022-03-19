// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetChassisPos;
import frc.robot.paths.Back;
import frc.robot.paths.Forward;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import raiderlib.geometry.Pose;
import frc.robot.commands.ChassisTurn180;
import frc.robot.commands.PursuitPath;

public class ShootLowGrabLow extends SequentialCommandGroup {
  /** Creates a new GrabBallToShoot. */
  public ShootLowGrabLow(Chassis chassis, Shooter shooter, Tower indexer, Intake intake, Limelight limelight, XboxController xboxController) {
    super(

      (new SetChassisPos(chassis, new Pose(0,0,0))),
      (new PursuitPath(chassis, new Back(), 45, 125, 10, 5, 10)),
      (new ChassisTurn180(chassis)),
      (new SetChassisPos(chassis, new Pose(0,0,Math.PI))),
      (new PursuitPath(chassis, new Forward(), 45, 125, 10, 5, 10))
    );
  }
}
