// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.k;


public class Shooter extends SubsystemBase {
  SparkMax m_shooterRightMotor = new SparkMax(k.SHOOTER.SHOOTER_RIGHT_MOTOR_CANID, MotorType.kBrushless);
  SparkMax m_shooterLeftMotor = new SparkMax(k.SHOOTER.SHOOTER_LEFT_MOTOR_CANID, MotorType.kBrushless);
  SparkMaxConfig m_shooterRightMotorConfig = new SparkMaxConfig();
  SparkMaxConfig m_shooterLeftMotorConfig = new SparkMaxConfig();
  double m_speed;
  /** Creates a new Intake. */
  public Shooter() {
    m_shooterRightMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.0)
    .voltageCompensation(12.8);
    m_shooterRightMotor.configure(m_shooterRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_shooterLeftMotorConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.0)
    .voltageCompensation(12.8);
    m_shooterLeftMotor.configure(m_shooterLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Voltage", m_speed);
  }
  /**
   * 
   * @param _speed +/- 1.0 
   */
  public void setSpeed(double _speed) {
    m_speed = _speed * k.ROBOT.MAX_BATTERY_VOLTAGE;
    m_shooterRightMotor.setVoltage(m_speed);
    m_shooterLeftMotor.setVoltage(m_speed);
  }
}