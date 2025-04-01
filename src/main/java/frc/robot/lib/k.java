// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class k {
    public static class OI{
        public static int driverController_port = 0;
        public static int operatorController_port = 1;

        public static CommandXboxController driverController = new CommandXboxController(driverController_port);
        public static Joystick operatorController = new Joystick(operatorController_port);

       // public static final Trigger DRIVER_MODE_TOGGLE = driverController.y();
        
    }
    public static class ROBOT{

        public static volatile Pose2d pose2d = new Pose2d();
        public static volatile Field2d field2d = new Field2d();
        public static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
        public static PowerDistribution pd = new PowerDistribution(1,ModuleType.kRev);
        public static final double MAX_BATTERY_VOLTAGE = 12.8;
        public static Drivetrain drive = new Drivetrain();
        public static Climber climber = new Climber();
        public static Shooter shooter = new Shooter();
    }
    public static class DRIVE{

        public static DriverModes driverMode = DriverModes.ARCADE;

        public static final int LEFT_MASTER_CANID = 1;
        public static final int LEFT_SLAVE_CANID = 2;
        public static final int RIGHT_MASTER_CANID = 3;
        public static final int RIGHT_SLAVE_CANID = 4;

        public static final double MOTOR_MAX_RPM = 5700;
        public static final double WHEEL_DIAMETER = 6;
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static final double GEAR_RATIO = 10.71;
    
        public static final double EncoderRevPerInch = GEAR_RATIO / WHEEL_CIRCUMFERENCE;
        public static final double MAX_VELOCITY = MOTOR_MAX_RPM / 60 * WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        public static final Vector<N3> STD_DEV_HIGH = VecBuilder.fill(0.15,0.15,0.15);
        public static final Vector<N3> STD_DEV_LOW = VecBuilder.fill(0.25,0.25,0.25);
    }
    public static class CLIMBER{
 
        public static final double GEAR_RATIO = 100;
        public static final double MAX_OUT_POSITION = 320;
        public static final double MAX_IN_POSITION = -250;
        public static final int CLIMBER_MOTOR_CANID = 9;
        public static final int CLIMBER_OUT_BUTTON_ID = 7;
        public static final int CLIMBER_IN_BUTTON_ID = 8;
        public static final int CLIMBER_SERVO_PWM = 2;

    }
    public static class SHOOTER{
        public static final int SHOOTER_RIGHT_MOTOR_CANID = 8;
        public static final int SHOOTER_LEFT_MOTOR_CANID = 7;
    }
}
