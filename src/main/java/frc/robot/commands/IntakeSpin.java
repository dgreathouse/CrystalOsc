// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.k;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeSpin extends Command {
  double m_speed;
  double m_timeout;
  Timer m_timer = new Timer();

  /**
   * 
   * @param _speed Speed from +/- 1.0
   * @param _timeout Timeout in seconds
   */
  public IntakeSpin(double _speed, double _timeout) {
    addRequirements(k.ROBOT.shooter);
    m_speed = _speed;
    m_timeout = _timeout; 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    k.ROBOT.shooter.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer.hasElapsed(m_timeout)){
      k.ROBOT.shooter.setSpeed(0);
      return true;
    } 
    return false;
  }
}
