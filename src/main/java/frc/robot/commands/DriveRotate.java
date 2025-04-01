// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.k;

public class DriveRotate extends Command {
  double m_angle;
  double m_speed;
  double m_timeOut;
  PIDController m_pid = new PIDController(.1, 0, 0);
  Timer m_timer = new Timer();
 
  
  public DriveRotate(double _angle, double _speed, double _timeout) {
    addRequirements(k.ROBOT.drive);
    m_angle = _angle;
    m_speed = _speed;
    m_timeOut = _timeout;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pid.reset();
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_pid.calculate(k.ROBOT.drive.getHeading(), m_angle);
    k.ROBOT.drive.arcadeDrive(0, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_pid.atSetpoint() || m_timer.hasElapsed(m_timeOut)){
      k.ROBOT.drive.arcadeDrive(0, 0);
      return true;
    }
    return false;
  }
}
