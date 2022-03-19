// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import raiderlib.control.DriveCharacterization;
import raiderlib.geometry.Pose;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */

  private WPI_TalonSRX m_leftMaster;
  private WPI_TalonSRX m_leftSlaveA;
  private WPI_TalonSRX m_leftSlaveB; 
  private MotorControllerGroup m_left;

  private WPI_TalonSRX m_rightMaster;
  private WPI_TalonSRX m_rightSlaveA;
  private WPI_TalonSRX m_rightSlaveB;
  private MotorControllerGroup m_right;

  // private DifferentialDrive m_drive;

  private final DifferentialDriveOdometry m_odometry;

  // private final DoubleSolenoid m_rightGearShifter;
  // private final DoubleSolenoid m_leftGearShifter;

  // private final Compressor pcmCompressor;

  private boolean shifterInHigh;

  private final AHRS m_navX;

  private final Pose m_currentPose;
  private final DriveCharacterization m_DriveCharacterization;

  // private MedianFilter lInput;
  // private MedianFilter rInput;

  // private double lEncoderCt;
  // private double rEncoderCt;

  // private double lPrevEncCt;
  // private double rPrevEncCt;

  // private double deltaLeft;
  // private double deltaRight;

  // private double lVelocity;
  // private double rVelocity;

  public Chassis() {

    // navX object instantiation
    m_navX = new AHRS(Port.kMXP);

    // Motor object instantiations, this year six Falcons were used
    m_leftMaster = new WPI_TalonSRX(Constants.CANID.kLeftMaster);
    m_leftSlaveA = new WPI_TalonSRX(Constants.CANID.kLeftSlaveA);
    m_leftSlaveB = new WPI_TalonSRX(Constants.CANID.kLeftSlaveB);
    m_left = new MotorControllerGroup(m_leftMaster, m_leftSlaveA, m_leftSlaveB);

    m_rightMaster = new WPI_TalonSRX(Constants.CANID.kRightMaster);
    m_rightSlaveA = new WPI_TalonSRX(Constants.CANID.kRightSlaveA);
    m_rightSlaveB = new WPI_TalonSRX(Constants.CANID.kRightSlaveB);
    m_right = new MotorControllerGroup(m_rightMaster, m_rightSlaveA, m_rightSlaveB);

    // m_drive = new DifferentialDrive(m_left, m_right);

    m_odometry = new DifferentialDriveOdometry(m_navX.getRotation2d());

    // Instantiation for Gear Shifter Double Solenoid Objects
    // m_leftGearShifter = new DoubleSolenoid(Constants.CANID.kPCM, PneumaticsModuleType.REVPH,
    //     Constants.PCM.kLeftGearForward, Constants.PCM.kLeftGearReverse);
    // m_rightGearShifter = new DoubleSolenoid(Constants.CANID.kPCM, PneumaticsModuleType.REVPH,
    //     Constants.PCM.kRightGearForward, Constants.PCM.kRightGearReverse);

    // pcmCompressor = new Compressor(Constants.CANID.kPCM, PneumaticsModuleType.REVPH);
    // pcmCompressor.enableAnalog(110, 120);

    m_currentPose = new Pose(0, 0, 0);

    // lInput = new MedianFilter(10);
    // rInput = new MedianFilter(10);

    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightSlaveA.setNeutralMode(NeutralMode.Brake);
    m_rightSlaveB.setNeutralMode(NeutralMode.Brake);

    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftSlaveA.setNeutralMode(NeutralMode.Brake);
    m_leftSlaveB.setNeutralMode(NeutralMode.Brake);

    m_rightMaster.setInverted(true);
    m_rightSlaveA.setInverted(true);
    m_rightSlaveB.setInverted(true);
    m_leftMaster.setInverted(false);
    m_leftSlaveA.setInverted(false);
    m_leftSlaveB.setInverted(false);

    m_leftSlaveA.follow(m_leftMaster);
    m_leftSlaveB.follow(m_leftMaster);
    m_rightSlaveA.follow(m_rightMaster);
    m_rightSlaveB.follow(m_rightMaster);


    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    m_leftMaster.config_kF(0, Constants.ChassisC.kF);
    m_rightMaster.config_kF(0, Constants.ChassisC.kF);

    m_leftMaster.config_kP(0, Constants.ChassisC.kP);
    m_rightMaster.config_kP(0, Constants.ChassisC.kP);

    // New drive characterization, all constants must be manually changed
    // in Constants.java class
    m_DriveCharacterization = new DriveCharacterization(Constants.ChassisC.maxVelocity,
        Constants.ChassisC.maxAcceleration,
        Constants.ChassisC.maxDecceleration, Constants.ChassisC.trackWidth);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(NativeToMeters(m_leftMaster.getSelectedSensorVelocity()),
        NativeToMeters(m_rightMaster.getSelectedSensorVelocity()));
  }

  public double getHeading() {
    return m_navX.getRotation2d().getDegrees();
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

   /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left.setVoltage(leftVolts);
    m_right.setVoltage(rightVolts);
    // m_drive.feed();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_navX.getRotation2d());
  }

  // sets speed of the chassis, takes input mode as a arugment to decrease error
  // during programming and to increase options for programming
  public void setSpeed(final Input_Mode im, double lSpeed, double rSpeed) {
    if (im == Input_Mode.Inches) {
      m_leftMaster.set(ControlMode.Velocity, InchesToNative(lSpeed));
      m_rightMaster.set(ControlMode.Velocity, InchesToNative(rSpeed));

    } else if (im == Input_Mode.PercentOut) {
      m_leftMaster.set(ControlMode.PercentOutput, lSpeed);
      m_rightMaster.set(ControlMode.PercentOutput, rSpeed);

    } else {
      m_leftMaster.set(ControlMode.Velocity, lSpeed);
      m_rightMaster.set(ControlMode.Velocity, rSpeed);
    }
  }

  public enum Input_Mode {
    PercentOut,
    Native,
    Inches
  }

  // Set current pose of the robot, to pass a pose as an argument do this:
  // setPose(mCurrPose.x, mCurrPose.y, mCurrPose.theta)
  public void setPose(double x, double y, double theta) {
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
    // lInput.reset();
    // rInput.reset();
    // m_navX.setAngleAdjustment(theta * (180 / Math.PI));
    m_currentPose.copy(x, y, theta);
  }

  public void resetEncoders() {
    m_leftMaster.setSelectedSensorPosition(0);
    m_leftSlaveB.setSelectedSensorPosition(0);
  }

  // Shifts gear box to higher setting
  // Make sure PCMA constant and gear shifter constants are correct
  // public void shiftToHigh() {
  //   m_leftGearShifter.set(Value.kForward);
  //   m_rightGearShifter.set(Value.kForward);
  //   shifterInHigh = true;
  // }

  // Shifts gear box to lower setting
  // Make sure PCMA constant and gear shifter constants are correct
  // public void shiftToLow() {
  //   m_leftGearShifter.set(Value.kReverse);
  //   m_rightGearShifter.set(Value.kReverse);
  //   shifterInHigh = false;
  // }

  // Helper Method, converts Inches to Encoder Count
  public double InchesToNative(double speed) {
    return speed / (2 * Math.PI * Constants.ShooterC.wheelRadius) * Constants.ShooterC.PPR / 10;
    // PPR is Pulses Per Rotation
    // divide by circ to get rotations, * PPR to get native units, /10 to go to
    // 100ms
  }

  // Helper Method, coverts encoder count to Inches
  public double NativeToInches(double speed) {
    return speed / Constants.ChassisC.PPR * Math.PI * 2 * Constants.ChassisC.wheelRadius * 10;
    // PPR is Pulses Per Rotation
    // divide by PPR to get rotations, * circ to get inches and * 10 to go 1s
  }

  // Helper Method, coverts encoder count speed to inches per second
  public double NativeToMeters(double speed) {
    return speed / Constants.ChassisC.PPR * Math.PI * 2 * Constants.ChassisC.wheelRadiusMeters * 10;
    // PPR is Pulses Per Rotation
    // divide by PPR to get rotations, * circ to get inches and * 10 to go 1s
  }

  public String getGearState() {
    if (shifterInHigh)
      return "High";
    else
      return "Low";
  }

  public DriveCharacterization getDriveCharacterization() {
    return m_DriveCharacterization;
  }

  public Pose getCurrentPose() {
    return m_currentPose;
  }

  public boolean getShifterState() {
    return shifterInHigh;
  }

  // public void turnChassis(double degrees) // turns the robot a certain heading
  // using imu
  // {
  // double t_leftSpeed;
  // double t_rightSpeed;
  // double t_error;
  // double t_kP = 0.2;
  // double t_adjP;
  // double yaw = m_currentPose.theta;
  // double target = yaw + degrees;
  // if (yaw > 180) {
  // yaw = yaw - 360;
  // }
  // int tolerance = 3;
  // while (Math.abs(yaw - target) > tolerance) {
  // if (yaw > 180) {
  // yaw = yaw - 360;
  // }
  // t_error = target - yaw;
  // t_adjP = t_error * t_kP;
  // t_leftSpeed = 50 + t_adjP;
  // t_rightSpeed = 50 - t_adjP;
  // if ((yaw - target) > 0) {
  // setSpeed(Input_Mode.PercentOut, -t_leftSpeed, t_rightSpeed);
  // } else {
  // setSpeed(Input_Mode.PercentOut, t_leftSpeed, -t_rightSpeed);
  // }
  // }
  // }

  @Override

  public void periodic() {
    // This method will be called once per scheduler run
    // lPrevEncCt = lEncoderCt;
    // rPrevEncCt = rEncoderCt;

    m_odometry.update(m_navX.getRotation2d(), NativeToMeters(m_leftMaster.getSelectedSensorPosition()),
        NativeToMeters(m_rightMaster.getSelectedSensorPosition()));
    // Uses MedianFilter to calcute the next position, using current position values
    // and median function
    // lEncoderCt = lInput.calculate(m_leftMaster.getSelectedSensorPosition());
    // rEncoderCt = rInput.calculate(m_rightMaster.getSelectedSensorPosition());

    // Change in encoder count from current position and expected next position
    // deltaLeft = lEncoderCt - lPrevEncCt;
    // deltaRight = rEncoderCt - rPrevEncCt;

    // Updating the current pose using navX angle and the delta left and right
    // calcuted
    // m_currentPose.update(m_navX.getAngle() * (Math.PI / 180),
    // NativeToInches(deltaLeft), NativeToInches(deltaRight));

    // Constantly updating left and right side velocity
    // lVelocity = m_leftMaster.getSelectedSensorVelocity();
    // rVelocity = m_rightMaster.getSelectedSensorVelocity();

    // Printing values to SmartDashboard, used during driving sessions
    // SmartDashboard.putNumber("x", m_currentPose.x);
    // SmartDashboard.putNumber("y", m_currentPose.x);
    // SmartDashboard.putNumber("angle", m_currentPose.theta);
    SmartDashboard.putString("Shifted State", getGearState());
    // SmartDashboard.putNumber("Pressure", pcmCompressor.getPressure());

    SmartDashboard.putNumber("LeftCount", m_leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("RightCount", m_rightMaster.getSelectedSensorPosition());
  }
}
