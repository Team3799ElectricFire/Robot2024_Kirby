// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.subsystems.Climber;

public class ClimberBothUp extends Command {
  private final Climber _climber;
  /** Creates a new climbArmUp. */
  public ClimberBothUp(Climber climber) {
    _climber =  climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Turn brakes off so climbers can move
    //_climber.leftBrakeOff();
    //_climber.rightBrakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If left climber is not at its' max height, move it up
    if (!_climber.leftAtMax()) {
      _climber.setLeftClimbMotor(MotorSpeeds.kClimberSpeed);
    } else {
      // Else, left climber IS at max height, stop moving it
      _climber.stopLeftClimbMotor();
    }

    // If right climber is not at its' max height, move it up
    if (!_climber.rightAtMax()) {
      _climber.setRightClimbMotor(MotorSpeeds.kClimberSpeed);
    } else {
      // Else, right climber IS at max height, stop moving it
      _climber.stopRightClimbMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop both climbers
    _climber.stopLeftClimbMotor();
    _climber.stopRightClimbMotor();

    // Put the brakes back on
    //_climber.leftBrakeOn();
    //_climber.rightBrakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
