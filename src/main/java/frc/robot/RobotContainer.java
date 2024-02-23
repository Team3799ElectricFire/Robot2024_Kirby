// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ShootInAmp;
import frc.robot.commands.ShootInSpeaker;
import frc.robot.commands.ClimberBothDown;
import frc.robot.commands.ClimberBothUp;
import frc.robot.commands.ClimberLeftDown;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeToShot;
import frc.robot.commands.ShootStop;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberRightDown;
import frc.robot.commands.DriveRobot;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Drivetrain drivetrain = new Drivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    Autos.init();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand(new DriveRobot(drivetrain, m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX));

    m_driverController.leftTrigger().whileTrue(new ShootInAmp(shooter));
    m_driverController.rightTrigger().whileTrue(new ShootInSpeaker(shooter));
    m_driverController.a().whileTrue(new IntakeToShot(intake));

    // m_driverController.leftTrigger().onTrue(new ShootInAmp(shooter));
    // m_driverController.rightTrigger().onTrue(new ShootInSpeaker(shooter));

    m_driverController.povUp().whileTrue(new ClimberBothUp(climber));
    m_driverController.povDown().whileTrue(new ClimberBothDown(climber));
    m_driverController.povLeft().whileTrue(new ClimberLeftDown(climber));
    m_driverController.povRight().whileTrue(new ClimberRightDown(climber));

    m_driverController.rightBumper().whileTrue(new IntakeIn(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.getAutonomousCommand();
  }
}
