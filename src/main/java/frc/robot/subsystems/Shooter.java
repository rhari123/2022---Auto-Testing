// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VerticalAngler;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final WPI_TalonFX m_ShootMotor1;
  private final WPI_TalonFX m_ShootMotor2;

  private final WPI_TalonSRX m_VertAngler;
  private final DigitalInput m_VertAngLimit;
  private final CANCoder m_VertEncoder;

  private final WPI_TalonSRX m_HorAngler;
  private final DigitalInput m_leftAngLimit;
  private final DigitalInput m_rightAngLimit;
  private final CANCoder m_HorEnconder;

  double shooterSpeed;
  double horizontalCt;
  double verticalCt;

  boolean atMaxLeft;
  boolean atMaxRight;

  boolean atMinVertical;

  public Shooter() {
    m_ShootMotor1 = new WPI_TalonFX(Constants.CANID.kShooter1);
    m_ShootMotor2 = new WPI_TalonFX(Constants.CANID.kShooter2);

    m_VertAngler = new WPI_TalonSRX(Constants.CANID.kVerticalAngler);
    m_VertAngLimit = new DigitalInput(Constants.DIO.kVerticalLimit);
    m_VertEncoder = new CANCoder(Constants.CANID.kVerticalCANCoder);

    m_HorAngler = new WPI_TalonSRX(Constants.CANID.kHorizontalAngler);
    m_leftAngLimit = new DigitalInput(Constants.DIO.kShooterLeftLimit);
    m_rightAngLimit = new DigitalInput(Constants.DIO.kShooterRightLimit);
    m_HorEnconder = new CANCoder(Constants.CANID.kHorizontalCANCoder);

    m_ShootMotor1.setNeutralMode(NeutralMode.Coast);
    m_ShootMotor2.setNeutralMode(NeutralMode.Coast);

    m_ShootMotor1.configClosedloopRamp(2);
    m_ShootMotor2.configClosedloopRamp(2);

    m_ShootMotor2.follow(m_ShootMotor1); 
    //This one is the follow, if works we can put it back, otherwise leave out

    m_ShootMotor2.setInverted(true);

    m_ShootMotor1.config_kF(0, Constants.ShooterC.kF);

    m_ShootMotor1.config_kP(0, Constants.ShooterC.kP);
  }

  public enum Input_Mode {
    PercentOut,
    Native,
    Inches
  }

  public enum Angle_Motor {
    ShootMotors,
    HorizontalAngler,
    VerticalAngler
  }

  public enum Angle_Unit {
    Degrees,
    Radians
  }

  public enum hoodPositions {
    Low, 
    Mid,
    High
  }

  public void setPosition(double target)
  {
    m_VertAngler.set(ControlMode.PercentOutput, (target - verticalCt) * Constants.VerticalAngler.anglerPIDkP);
  }

  public void setSpeed(final Input_Mode im, final Angle_Motor am, double speed) {
    switch (am) {
      case ShootMotors:
        if (im == Input_Mode.Inches) {
          m_ShootMotor1.set(ControlMode.Velocity, InchesToNative(Angle_Motor.ShootMotors, speed));
          //Master Slave

          // m_ShootMotor1.set(ControlMode.Velocity, InchesToNative(Angle_Motor.ShootMotors, speed));
          // m_ShootMotor2.set(ControlMode.Velocity, InchesToNative(Angle_Motor.ShootMotors, speed));
          break;
        } else if (im == Input_Mode.PercentOut) {
          m_ShootMotor1.set(ControlMode.PercentOutput, speed); 
          //Master Slave

          // m_ShootMotor1.set(ControlMode.PercentOutput, speed);
          // m_ShootMotor2.set(ControlMode.PercentOutput, speed);
          break;
        } else {
          m_ShootMotor1.set(ControlMode.Velocity, speed);
          //Master Slave

          // m_ShootMotor1.set(ControlMode.Velocity, speed);
          // m_ShootMotor2.set(ControlMode.Velocity, speed);
          break;
        }

      case VerticalAngler:
        if (im == Input_Mode.Inches) {
        m_VertAngler.set(ControlMode.Velocity,
        InchesToNative(Angle_Motor.VerticalAngler, speed));
        break;
        } else if (im == Input_Mode.PercentOut) {
        m_VertAngler.set(ControlMode.PercentOutput, speed);
        break;
        } else {
        m_VertAngler.set(ControlMode.Velocity, speed);
        break;
        }

      case HorizontalAngler:
        if (im == Input_Mode.Inches) {
        m_HorAngler.set(ControlMode.Velocity,
        InchesToNative(Angle_Motor.HorizontalAngler, speed));
        break;
        } else if (im == Input_Mode.PercentOut) {
        m_HorAngler.set(ControlMode.PercentOutput, speed);
        break;
        } else {
        m_HorAngler.set(ControlMode.Velocity, speed);
        break;
        }
    }
  }

  // sets the angle of the motor, either horizontal or vertical angler, given the
  // unit of the angle
  // itself
  // public void setAngle(final Angle_Motor am, final Angle_Unit au, double angle) {
  //   switch (am) {
  //     case HorizontalAngler:
  //       if (au == Angle_Unit.Degrees) {
  //       m_HorAngler.set(ControlMode.Position, (angle) / 360 *
  //       Constants.HorizontalAngler.PPR);
  //       }

  //       else {
  //       m_HorAngler.set(ControlMode.Position, (angle) / (Math.PI * 2) *
  //       }

  //     case VerticalAngler:
  //       if (au == Angle_Unit.Degrees) {
  //       m_VertAngler.set(ControlMode.Position, (angle) / 360 *
  //       Constants.VerticalAngler.PPR);
  //       } else {
  //       m_VertAngler.set(ControlMode.Position, (angle) / (Math.PI * 2) *
  //       Constants.VerticalAngler.PPR);
  //       }

  //     case ShootMotors:
  //   }

  // }

  public double InchesToNative(Angle_Motor am, double speed) {
    if (am == Angle_Motor.ShootMotors) {
      return speed / (2 * Math.PI * Constants.ShooterC.wheelRadius) * Constants.ShooterC.PPR / 10;
    } else if (am == Angle_Motor.HorizontalAngler) {
      return speed / (2 * Math.PI * Constants.HorizontalAngler.radius) * Constants.HorizontalAngler.PPR / 10;
    } else {
      return speed / (2 * Math.PI * Constants.VerticalAngler.radius) * Constants.VerticalAngler.PPR / 10;
    }

    // divide by circ to get rotations, * PPR to get native units, /10 to go to
    // 100ms
  }

  public double NativeToInches(Angle_Motor am, double speed) {
    if (am == Angle_Motor.ShootMotors) {
      return speed / Constants.ShooterC.PPR * Math.PI * 2 * Constants.ShooterC.wheelRadius * 10;
    } else if (am == Angle_Motor.HorizontalAngler) {
      return speed / Constants.ShooterC.PPR * Math.PI * 2 * Constants.HorizontalAngler.radius * 10;
    } else {
      return speed / Constants.ShooterC.PPR * Math.PI * 2 * Constants.VerticalAngler.radius * 10;
    }
    // divide by PPR to get rotations, * circ to get inches and * 10 to go 1s
  }

  public boolean atLeftLimit() {
    return atMaxLeft;
  }

  public boolean atRightLimit() {
    return atMaxRight;
  }

  public boolean atVerticalLimit() {
    return atMinVertical;
  }

  public double getHorizontalCt() {
    return horizontalCt;
  }
  public double getVerticalCt() {
    return verticalCt;
  }

  public void setVerticalAngle(double angle) {
    verticalCt = angle;
  }

  public double getShooterSpeed(){
    return m_ShootMotor1.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterSpeed = m_ShootMotor1.getSelectedSensorVelocity();

    atMinVertical = m_VertAngLimit.get();
    atMaxLeft = m_leftAngLimit.get();
    atMaxRight = m_rightAngLimit.get();

    horizontalCt = m_HorEnconder.getPosition();

    verticalCt = -m_VertEncoder.getPosition();

    // if (atMaxLeft) {
    // m_HorAngler.setSelectedSensorPosition(Constants.HorizontalAngler.kLeftOffset
    // / 360 * Constants.HorizontalAngler.PPR);
    // }

    // if (atMaxRight) {
    // m_HorAngler.setSelectedSensorPosition(Constants.HorizontalAngler.kRightOffset
    // / 360 * Constants.HorizontalAngler.PPR);
    // }

    atMinVertical = m_VertAngLimit.get();
    atMaxLeft = m_leftAngLimit.get();
    atMaxRight = m_rightAngLimit.get();

    if (atMaxLeft == true || atMaxRight == true ){
      m_HorAngler.set(0);
    }

    if (atMinVertical == true || verticalCt >= 29500){
      m_VertAngler.set(0);
    }
    

    SmartDashboard.putBoolean("At Min Vertical", atMinVertical);
    SmartDashboard.putBoolean("Max Left", atMaxLeft);
    SmartDashboard.putBoolean("Max Right", atMaxRight);

    // SmartDashboard.putNumber("Horizontal Angle", horizontalAngle);

    SmartDashboard.putNumber("vertAng", verticalCt);
    SmartDashboard.putNumber("horAng", horizontalCt);

    SmartDashboard.putBoolean("Ready To Shoot", Constants.ShooterC.isReady);
  }
}
