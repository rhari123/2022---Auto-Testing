// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final WPI_TalonFX m_sExtender;
  private final WPI_TalonFX m_Extender;
  private final DigitalInput downLimitA;
  private final DigitalInput downLimitB;
  private final DigitalInput upperLimitA;
  private final DigitalInput upperLimitB;

  private double eEncoderCt;
  private boolean isMax = false;
  private boolean isMin = false;

  /** Creates a new Climber. */
  public Climber() {
    m_sExtender = new WPI_TalonFX(Constants.CANID.kClimberExtenderS);
    m_Extender = new WPI_TalonFX(Constants.CANID.kClimberExtenderM);

    m_sExtender.follow(m_Extender);

    m_sExtender.setNeutralMode(NeutralMode.Brake);
    m_Extender.setNeutralMode(NeutralMode.Brake);

    m_sExtender.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_Extender.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // m_Extender.config_kF(0, Constants.ClimberC.kF);
    m_Extender.config_kP(0, Constants.ClimberC.kP);

    downLimitA = new DigitalInput(Constants.DIO.kLeftMin);
    upperLimitA = new DigitalInput(Constants.DIO.kLeftMax);

    downLimitB = new DigitalInput(Constants.DIO.kRightMin);
    upperLimitB = new DigitalInput(Constants.DIO.kRightMax);

  }

  boolean climberIsMax() {
    return isMax;
  }

  boolean climberIsMin() {
    return isMin;
  }

  void extend() {
    //m_Extender.set(ControlMode.Position, Constants.ClimberC.extenderLimit);
    isMax = true;
  }

  boolean atUpperLimit() {
    return (upperLimitA.get() && upperLimitB.get());
  }

  boolean atLowerLimit() {
    return (downLimitA.get() && downLimitB.get());
  }

  @Override
  public void periodic() {

    // Stop motors if the upper limit, lower limit, or soft encoder limit are true
    // if (atUpperLimit() || atLowerLimit() || eEncoderCt >= Constants.ClimberC.extenderLimit) {
    //   m_Extender.set(ControlMode.PercentOutput, 0);
    // }
  }
}
