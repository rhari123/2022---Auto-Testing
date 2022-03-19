// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tower extends SubsystemBase {
  /** Creates a new Indexer. */
  private final WPI_VictorSPX m_indexerTower;
  private final DigitalInput m_digEnt;
  private final DigitalInput m_digEx;

  private boolean occupied; 

  private boolean prevEntState; 
  private boolean currEntState; 

  private boolean prevExitState; 
  private boolean currExitState; 

  
  public Tower() {
    m_indexerTower = new WPI_VictorSPX(Constants.CANID.kIndexerTower);
    m_digEnt = new DigitalInput(Constants.DIO.kTowerIn);
    m_digEx = new DigitalInput(Constants.DIO.kTowerOut);

    m_indexerTower.setInverted(true);

    prevEntState = false; 
    prevExitState = false; 
  }

  public void towerUp() {
    m_indexerTower.set(0.8);
  }

  public void towerDown() {
    m_indexerTower.set(-0.8);
  }

  public void towerStop(){
    m_indexerTower.set(0);
  }

  public void towerIdle(){
    m_indexerTower.set(0.35);
  }

  public boolean isOccupied(){
    return occupied; 
  }

  public boolean getEnt(){
    return m_digEnt.get();
  }

  public boolean getEx(){
    return m_digEx.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // currEntState = m_digEnt.get();
    currExitState = m_digEx.get();

    if (!prevEntState && currEntState){
      occupied = true;
    }

    if (!prevExitState && currExitState){
      occupied = false;
    }

    prevEntState = currEntState;
    prevExitState = currExitState;
    SmartDashboard.putBoolean("Ball Ent", m_digEnt.get());
    SmartDashboard.putBoolean("Ball Exit", m_digEx.get()); 
  }
}
