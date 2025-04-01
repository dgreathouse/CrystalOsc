// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.k;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterDefaultCommand extends Command {
  /** Creates a new IntakeDefaultCommand. */
  public ShooterDefaultCommand() {
    addRequirements(k.ROBOT.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = 0;
    if(k.OI.operatorController.getRawButton(1)){
      speed = 0.1;
    }else if(k.OI.operatorController.getRawButton(2)){
      speed = -0.1;
    }else if(k.OI.operatorController.getRawButton(3)){
      speed = 0.4;
    }else if(k.OI.operatorController.getRawButton(4)){
      speed = -0.4;
    }
    k.ROBOT.shooter.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
