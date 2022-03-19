// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoBackFor5;
import frc.robot.commands.ShootAndMoveBackPar;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAndMoveBack extends SequentialCommandGroup {
  /** Creates a new ShootAndMoveBack. */
  public ShootAndMoveBack(Chassis chassis, Shooter shooter, Tower tower) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //Simple shoot for a certain amount of time and move back, 
    //worked at Bridgewater-Raritan District Competition, shot
    //low and moved back for 5 seconds

    super(
      (new ShootAndMoveBackPar(tower, shooter)),
      (new GoBackFor5(chassis))
    );
  }
}
