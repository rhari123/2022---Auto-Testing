// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.CameraServo;
import frc.robot.subsystems.Chassis;
import frc.robot.commands.DefaultTower;
import frc.robot.commands.DefaultIntake;
import frc.robot.commands.DefaultShooter;
import frc.robot.commands.DefaultTempClimber;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.JoystickDriving;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ToggleGear;
import frc.robot.commands.ToggleIntakeLifter;
import frc.robot.commands.ToggleServo;
import frc.robot.commands.autos.ShootAndMoveBack;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TempClimber;
import frc.robot.commands.FollowPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final Shooter m_shooter = new Shooter();
  // private final Tower m_tower = new Tower();
  // private final Intake m_intake = new Intake();
  private final Chassis m_chassis = new Chassis(); 
  // private final Limelight m_limelight = new Limelight();
  // private final CameraServo m_servo = new CameraServo();

  // private final TempClimber m_climber = new TempClimber();

  private final Joystick m_leftStick = new Joystick(Constants.Joystick.leftStick);
  private final Joystick m_rightStick = new Joystick(Constants.Joystick.rightStick);
  private final XboxController m_xboxController = new XboxController(2);  

  public final JoystickButton m_turn180 = new JoystickButton(m_leftStick, Constants.Joystick.turnButton); //testing

  private final JoystickButton m_gearShiftButton = new JoystickButton(m_leftStick, Constants.Joystick.gearShiftButton);
  private final JoystickButton m_intakeOutButton = new JoystickButton(m_leftStick, Constants.Joystick.intakeOutButton);
  private final Button m_toggleIntakeButton = new Button(() -> m_xboxController.getLeftBumper());

  private final Button m_intakeBallButton = new Button(() -> m_xboxController.getLeftTriggerAxis() > 0.75);

  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

      m_chassis.setDefaultCommand(new JoystickDriving(m_chassis, m_leftStick, m_rightStick));
      // m_shooter.setDefaultCommand(new DefaultShooter(m_shooter, m_xboxController));
      // m_intake.setDefaultCommand(new DefaultIntake(m_intake, m_xboxController));
      // m_tower.setDefaultCommand(new DefaultTower(m_tower, m_xboxController));
      // m_climber.setDefaultCommand(new DefaultTempClimber(m_climber, m_xboxController));

      // m_servo.setDefaultCommand(new ToggleServo(m_servo, m_leftStick));



    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //m_gearShiftButton.whenPressed(new ToggleGear(m_chassis));
    // m_intakeBallButton.whenPressed(new IntakeBall(m_intake));
    // m_toggleIntakeButton.whenPressed(new ToggleIntakeLifter(m_intake));
    // m_intakeOutButton.whenPressed(new ReverseIntake(m_intake, m_tower, m_leftStick));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // public Command getAutonomousCommand() {
  //   return m_autoCommand;
  // }

  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.Autos.kS,
                Constants.Autos.kV,
                Constants.Autos.kA),
            Constants.Autos.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.Autos.maxSpeed,
                Constants.Autos.maxAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Autos.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            // exampleTrajectory,
            Constants.Autos.trajectory,
            m_chassis::getPose,
            new RamseteController(Constants.Autos.kRamseteB, Constants.Autos.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.Autos.kS,
                Constants.Autos.kV,
                Constants.Autos.kA),
            Constants.Autos.kDriveKinematics,
            m_chassis::getWheelSpeeds,
            new PIDController(Constants.Autos.kP, 0, 0),
            new PIDController(Constants.Autos.kP, 0, 0),
            // RamseteCommand passes volts to the callback
            m_chassis::tankDriveVolts,
            m_chassis);

    // Reset odometry to the starting pose of the trajectory.
    m_chassis.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_chassis.tankDriveVolts(0, 0));
  }
}