// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.k;

public class DrivetrainDefaultCommand extends Command {
  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand() {
    addRequirements(k.ROBOT.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    k.ROBOT.drive.m_robotDrive.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = -k.OI.driverController.getRawAxis(1);
    double rotate = -k.OI.driverController.getRawAxis(2);

    forward = squareInput(forward);
    rotate = squareInput(rotate);



    rotate = squareInput(rotate);
    forward = squareInput(forward);
    
    switch(k.DRIVE.driverMode){
      case ARCADE:
        k.ROBOT.drive.arcadeDrive(forward, rotate);
        break;
      case TANK:
      //  k.ROBOT.drive.tankDrive(left, right);
        break;
      case CURVATURE:
       // k.ROBOT.drive.curvatureDrive(forward, rotate, true);
        break;
      case PID:
        break;
    }
  }
  private double squareInput(double _input){
    return Math.copySign(_input * _input, _input);
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
