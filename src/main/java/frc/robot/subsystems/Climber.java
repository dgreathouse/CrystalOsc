// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.k;

public class Climber extends SubsystemBase {
  SparkMax m_climberMotor = new SparkMax(k.CLIMBER.CLIMBER_MOTOR_CANID, MotorType.kBrushless);
  SparkMaxConfig m_climberMotorConfig = new SparkMaxConfig();

  public Climber() {
    m_climberMotorConfig.inverted(false)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.25)
    .voltageCompensation(12.8);
    m_climberMotor.configure(m_climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  }
  public void setVoltage(double _voltage){
    if(getPosition() > k.CLIMBER.MAX_OUT_POSITION && _voltage > 0){
      _voltage = 0;
    }
    if(getPosition() < k.CLIMBER.MAX_IN_POSITION && _voltage < 0){
      _voltage = 0;
    }
    m_climberMotor.setVoltage(_voltage);
  }
  
  public double getPosition(){
    return m_climberMotor.getEncoder().getPosition();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Position", m_climberMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber/AppliedVoltage", m_climberMotor.getAppliedOutput());
    SmartDashboard.putNumber("Climber/OutputCurrent", m_climberMotor.getOutputCurrent());
  }
}
