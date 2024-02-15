// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeIn extends Command {
  private final Intake _Intake;

  /** Creates a new IntakeIn. */
  public IntakeIn(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    _Intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!_Intake.haveNote()){
      _Intake.runIntake();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _Intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _Intake.haveNote();
  }
}
