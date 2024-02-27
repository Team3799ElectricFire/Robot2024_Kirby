// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class supplyFromHumanStation extends Command {
  private final Intake _Intake;
  private final Shooter _Shooter;
  /** Creates a new supplyFromHumanStation. */
  public supplyFromHumanStation(Intake intake, Shooter shooter) {
    _Intake = intake; 
    _Shooter = shooter;
    addRequirements(intake, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _Shooter.supplyFromHumanStation();
    _Intake.ejectIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _Shooter.shootStop();
    _Intake.stopIntake();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _Intake.haveNote();
  }
}
