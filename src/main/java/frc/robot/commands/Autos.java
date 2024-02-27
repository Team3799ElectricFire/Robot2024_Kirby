// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /* Driverstation Chooser for autonomous commands */
  private static final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static void init() {
    // Setup autoChooser
    autoChooser.setDefaultOption("None",none());
    // ...add all Auto options here...
    // autoChooser.addOption("New Auto", new AutoCommand());

    autoChooser.addOption("Test1", new PathPlannerAuto("Test1"));

    // Put the chooser on the driver station
    SmartDashboard.putData("Autonomous Mode", autoChooser);
  }

  public static Command none() {
    return Commands.none();
  }

public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
}


  // Create more auto commands down here...
  /* public static Command AutoCommand() {
   *
   * }
   */
}
