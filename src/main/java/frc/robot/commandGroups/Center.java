// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveRotate;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.IntakeSpin;


public class Center extends SequentialCommandGroup {
  /** Creates a new Center. */
  public Center() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToDistance(1, .5, 2),
      new IntakeSpin(.5, 2),
      new DriveToDistance(-.5, .5, 2),
      new DriveRotate(45, .5, 1)
    );
  }
}
