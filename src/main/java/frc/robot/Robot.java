// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commandGroups.Center;
import frc.robot.commandGroups.DoNothing;
import frc.robot.commands.ClimberDefaultCommand;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.lib.k;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand();
  private ClimberDefaultCommand m_climberDefaultCommand = new ClimberDefaultCommand();
  private ShooterDefaultCommand m_shooterDefaultCommand = new ShooterDefaultCommand();
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private UsbCamera usbCamera_1 = CameraServer.startAutomaticCapture(0);
  private UsbCamera usbCamera_2 = CameraServer.startAutomaticCapture(1);
  private UsbCamera usbCamera_3 = CameraServer.startAutomaticCapture(2);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    k.ROBOT.drive.setDefaultCommand(m_drivetrainDefaultCommand);
    k.ROBOT.climber.setDefaultCommand(m_climberDefaultCommand);
    k.ROBOT.shooter.setDefaultCommand(m_shooterDefaultCommand);
    m_autoChooser.setDefaultOption("Do Nothing", new DoNothing());
    m_autoChooser.addOption("Center", new Center());
    SmartDashboard.putData(m_autoChooser);

    usbCamera_1.setResolution(120, 80);
    usbCamera_2.setResolution(120, 80);
    usbCamera_3.setResolution(120, 80);
    usbCamera_1.setFPS(10);
    usbCamera_2.setFPS(10);
    usbCamera_3.setFPS(10);

  }
  public void configureButtonBindings() {
  //  k.OI.DRIVER_MODE_TOGGLE.onTrue(new InstantCommand(()-> {k.ROBOT.drive.toggleThroughDriveModes();}, k.ROBOT.drive ));

  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    k.ROBOT.gyro.reset();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
