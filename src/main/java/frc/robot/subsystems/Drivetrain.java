// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.DriverModes;
import frc.robot.lib.k;

public class Drivetrain extends SubsystemBase {
  SparkMax m_leftMaster = new SparkMax(k.DRIVE.LEFT_MASTER_CANID, MotorType.kBrushless);
  SparkMax m_leftSlave = new SparkMax(k.DRIVE.LEFT_SLAVE_CANID, MotorType.kBrushless);
  SparkMax m_rightMaster = new SparkMax(k.DRIVE.RIGHT_MASTER_CANID, MotorType.kBrushless);
  SparkMax m_rightSlave = new SparkMax(k.DRIVE.RIGHT_SLAVE_CANID, MotorType.kBrushless);
  
  MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMaster, m_leftSlave);
  MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMaster, m_rightSlave);
  SparkMaxConfig m_leftMasterConfig = new SparkMaxConfig();
  SparkMaxConfig m_leftSlaveConfig = new SparkMaxConfig();
  SparkMaxConfig m_rightMasterConfig = new SparkMaxConfig();
  SparkMaxConfig m_rightSlaveConfig = new SparkMaxConfig();

  public DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotors::set, m_rightMotors::set);
  DifferentialDrivePoseEstimator m_poseEstimator;
  DifferentialDriveKinematics m_kinematics;
  PoseEstimatorThread m_poseEstimatorThread;

  public Drivetrain() {
    m_leftMasterConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.25)
    .voltageCompensation(12.8);
    m_leftMaster.configure(m_leftMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_leftSlaveConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.25)
    .voltageCompensation(12.8);
    m_leftSlave.configure(m_leftSlaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_rightMasterConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.25)
    .voltageCompensation(12.8);
    m_rightMaster.configure(m_rightMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_rightSlaveConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.25)
    .voltageCompensation(12.8);
    m_rightSlave.configure(m_rightSlaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_kinematics = new DifferentialDriveKinematics(.5);
    m_poseEstimator = new DifferentialDrivePoseEstimator(
     m_kinematics,
      k.ROBOT.gyro.getRotation2d(),
      getLeftDistance(),
      getRightDistance(),
      new Pose2d()
    );

    m_poseEstimatorThread = new PoseEstimatorThread();
    m_poseEstimatorThread.start();


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive/LeftPos", getLeftDistance());
    SmartDashboard.putNumber("Drive/RightPos", getRightDistance());
    SmartDashboard.putData("Drive/Field", k.ROBOT.field2d);
    SmartDashboard.putNumber("Drive/Heading", getHeading());
    SmartDashboard.putData("Drive/DifferentialDrive", m_robotDrive);
   
  }
  public double getDistance(){
    return ((m_leftMaster.getEncoder().getPosition()/k.DRIVE.EncoderRevPerInch) + m_rightMaster.getEncoder().getPosition()/k.DRIVE.EncoderRevPerInch) / 2;
  }
  public double getLeftDistance(){
    return  m_leftMaster.getEncoder().getPosition()/17.9;
  }
  public double getRightDistance(){
    return -m_rightMaster.getEncoder().getPosition()/17.9;
  }
  public double getLeftVelocity(){
    return m_leftMaster.getEncoder().getVelocity();
  }
  public double getRightVelocity(){
    return m_rightMaster.getEncoder().getVelocity();
  }
  public void resetPositions(){
    m_leftMaster.getEncoder().setPosition(0);
    m_rightMaster.getEncoder().setPosition(0);
  }
  public void arcadeDrive(double forward, double rotate){
    m_robotDrive.arcadeDrive(forward, rotate);
  }
  public void tankDrive(double left, double right){
    m_robotDrive.tankDrive(left, right);
  }
  public void curvatureDrive(double forward, double rotate, boolean allowTurnInPlace){
    m_robotDrive.curvatureDrive(forward, rotate, allowTurnInPlace);
  }
  public void setVoltage(double left, double right){
    m_leftMaster.setVoltage(left);
    m_rightMaster.setVoltage(right);
  }
  public void toggleThroughDriveModes(){
    switch(k.DRIVE.driverMode){
      case ARCADE:
        k.DRIVE.driverMode = DriverModes.TANK;
        break;
      case TANK:
        k.DRIVE.driverMode = DriverModes.CURVATURE;
        break;
      case CURVATURE:
        k.DRIVE.driverMode = DriverModes.ARCADE;
        break;
      default:
        break;
    }
  }
  public void stop(){
    m_robotDrive.stopMotor();
  }

  public double getHeading() {
    
    return k.ROBOT.gyro.getAngle();
  }
  public Rotation2d getHeading2d(){
    return k.ROBOT.gyro.getRotation2d();
  }
  private class PoseEstimatorThread extends Thread {
    public PoseEstimatorThread() {
      super();
    }

    @Override
    public void run() {
      while (true) {

        k.ROBOT.pose2d = m_poseEstimator.update(
          getHeading2d(),
          getLeftDistance(),
          getRightDistance()
        );
        k.ROBOT.field2d.setRobotPose(k.ROBOT.pose2d);
        try {
          Thread.sleep(5);
        } catch (InterruptedException e) {
          System.out.println(e.getMessage());
        }
      }
    }
  }

}
