// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.k;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToDistance extends Command {
  double m_distance_in;
  double m_speed;
  double m_timeout;
  PIDController m_rotatePID = new PIDController(0.1, 0, 0);
  PIDController m_drivePID = new PIDController(0.1, 0, 0);
  double m_initialAngle;
  
  Timer m_timer = new Timer();

  /** Creates a new DriveToDistance. */
  public DriveToDistance(double _distance_in, double _speed, double _timeout) {
    addRequirements(k.ROBOT.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    k.ROBOT.drive.resetPositions();
    m_rotatePID.reset();
    m_rotatePID.setTolerance(1);
    m_initialAngle = k.ROBOT.drive.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotate = 0;
    double angle = k.ROBOT.drive.getHeading();
    rotate = m_rotatePID.calculate(angle, m_initialAngle);

    double speed = m_speed;
    double distance = k.ROBOT.drive.getDistance();
    speed = m_drivePID.calculate(distance, m_distance_in);
    speed = rampUpValue(m_speed, .5); 

    k.ROBOT.drive.tankDrive(speed + rotate, speed - rotate);
  }
  private double rampUpValue(double _val, double _rampTime_sec) {
    double currentTime_sec = m_timer.get();
    if (currentTime_sec < _rampTime_sec) {
      _val = _val * currentTime_sec / _rampTime_sec;
    }
    return _val;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_timeout) || m_drivePID.atSetpoint()) {
      k.ROBOT.drive.tankDrive(0, 0);
      return true;
    }
    return false;
  }
}
