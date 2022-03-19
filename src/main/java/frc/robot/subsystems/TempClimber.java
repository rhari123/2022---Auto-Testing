// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberC;

public class TempClimber extends SubsystemBase {
  private final WPI_TalonFX m_sExtender;
  private final WPI_TalonFX m_Extender;

  private double leftCount;
  private double rightCount;

  private boolean goingUp;

  private double climberCt ;
  /** Creates a new TempClimber. */
  public TempClimber() {
    m_sExtender = new WPI_TalonFX(Constants.CANID.kClimberExtenderS); //Left
    m_Extender = new WPI_TalonFX(Constants.CANID.kClimberExtenderM); //Right

    // m_sExtender.follow(m_Extender);

    m_sExtender.setNeutralMode(NeutralMode.Brake);
    m_Extender.setNeutralMode(NeutralMode.Brake);

    m_sExtender.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_Extender.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // m_Extender.config_kF(0, Constants.ClimberC.kF);
    m_Extender.config_kP(0, Constants.ClimberC.kP);
    m_Extender.config_kD(0,Constants.ClimberC.kD);

    // m_sExtender.config_kF(0, Constants.ClimberC.kF);
    m_sExtender.config_kP(0, Constants.ClimberC.kP);
    m_sExtender.config_kD(0,Constants.ClimberC.kD);

  }

  public void up(){
    goingUp = true;
    // m_Extender.set(ControlMode.Position, Constants.ClimberC.upperLimit);
    // m_sExtender.set(ControlMode.Position, Constants.ClimberC.upperLimit);

    m_Extender.set(0.3);
    m_sExtender.set(0.3);
    
  }

  public void down(){
    goingUp = false;
    // m_Extender.set(ControlMode.Position, Constants.ClimberC.lowerLimit);
    // m_sExtender.set(ControlMode.Position, Constants.ClimberC.lowerLimit);

    m_Extender.set(-0.3);
    m_sExtender.set(-0.3);
  }

  public void stop(){
    m_Extender.set(0);
    m_sExtender.set(0);
  }

  //1 is left, 2 is right
  public double getEncCt(int side){
    if (side == 1){
      return m_Extender.getSelectedSensorPosition();
    }

    else {
      return m_sExtender.getSelectedSensorPosition();
    }
  }

  public void setPosition (double ct){
    m_Extender.setSelectedSensorPosition(ct);
    m_sExtender.setSelectedSensorPosition(ct);
  }

  public double getLeftCt(){
    return leftCount;
  }

  public double getRightCt(){
    return rightCount;
  }

  public void robotUp(){
    m_Extender.set(-0.72);
    m_sExtender.set(-0.8); // left
    goingUp = false;
    // m_Extender.set(-0.15);
    // m_sExtender.set(-0.20);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    if (leftCount < 0 && !goingUp){
      m_sExtender.set(0);
    }

    if (leftCount >= 101000 && goingUp){
      m_sExtender.set(0);
    }

    if (rightCount < 0 && !goingUp ){
      m_Extender.set(0);
    }

    if (rightCount >=101000 && goingUp){
      m_Extender.set(0);
    }

  SmartDashboard.putNumber("ClimberLeftCt",m_sExtender.getSelectedSensorPosition()); 
  SmartDashboard.putNumber("ClimberRightCt",m_Extender.getSelectedSensorPosition()); 

  leftCount = m_sExtender.getSelectedSensorPosition();
  rightCount = m_Extender.getSelectedSensorPosition();
  }
}
