// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootInSpeaker extends Command {
  private final Shooter _speakerShooter;
  /** Creates a new shootSpeaker. */
  public ShootInSpeaker(Shooter speakerShoota) {
    _speakerShooter = speakerShoota;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(speakerShoota);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    _speakerShooter.prepShootAtSpeaker(); // TODO check speed 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  // TODO check what to dp if interupted and why red
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
