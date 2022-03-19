// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final WPI_VictorSPX m_intake;

  private final DoubleSolenoid m_rightIntakeLifter;
  private final DoubleSolenoid m_leftIntakeLifter;

  boolean pneumaticsEngaged; 

  /** Creates a new Intake. */
  public Intake() {
    m_intake = new WPI_VictorSPX(Constants.CANID.kIntake);

    m_leftIntakeLifter = new DoubleSolenoid(Constants.CANID.kPCM, PneumaticsModuleType.REVPH,
        Constants.PCM.kRightIntakeLifterForward, Constants.PCM.kRightIntakeLifterReverse);
    m_rightIntakeLifter = new DoubleSolenoid(Constants.CANID.kPCM, PneumaticsModuleType.REVPH,
        Constants.PCM.kLeftIntakeLifterForward, Constants.PCM.kLeftIntakeLifterReverse);
  }

  public void stop(){
    m_intake.set(0);
  }

  public void up(){
    m_leftIntakeLifter.set(Value.kReverse);
    m_rightIntakeLifter.set(Value.kReverse);
    pneumaticsEngaged = true;
  }

  public void down(){
    m_rightIntakeLifter.set(Value.kForward);
    m_leftIntakeLifter.set(Value.kForward);
    pneumaticsEngaged = false;
  }

  public void toggle(){
    if(pneumaticsEngaged){
      m_leftIntakeLifter.set(Value.kReverse);
      m_rightIntakeLifter.set(Value.kReverse);
      pneumaticsEngaged = true;
    }
    
    else{
      m_rightIntakeLifter.set(Value.kForward);
      m_leftIntakeLifter.set(Value.kForward);
      pneumaticsEngaged = false;
    }
  }

  public void takeIn() {
    m_intake.set(0.9);
  }

  public boolean getEngagedState(){
    return pneumaticsEngaged;
  }

  public void ballOut() {
    m_intake.set(-0.3);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
