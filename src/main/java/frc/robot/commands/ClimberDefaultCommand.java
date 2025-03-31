// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.k;

public class ClimberDefaultCommand extends Command {
  /** Creates a new ClimberDefaultCommand. */
  public ClimberDefaultCommand() {
    addRequirements(k.ROBOT.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(k.OI.operatorController.getRawButton(k.CLIMBER.CLIMBER_OUT_BUTTON_ID)){
      k.ROBOT.climber.setVoltage(2.0);
    }else if(k.OI.operatorController.getRawButton(k.CLIMBER.CLIMBER_IN_BUTTON_ID)){
      k.ROBOT.climber.setVoltage(-2.0);
    }else{
      k.ROBOT.climber.setVoltage(0);
    }
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
