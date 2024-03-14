// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberSticks extends Command {
  private final Climber _Climber;
  private DoubleSupplier _LeftSupplier, _RightSupplier;

  /** Creates a new ClimberSticks. */
  public ClimberSticks(Climber climber, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
    _Climber = climber;
    _LeftSupplier = leftSupplier;
    _RightSupplier = rightSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Left Climber
    double leftDemand = -0.5 * _LeftSupplier.getAsDouble();
    if ((Math.abs(leftDemand) > Constants.minThumbstickMagnitude) &&
        ((leftDemand > 0 && !_Climber.leftAtMax()) || (leftDemand < 0 && !_Climber.leftAtHome()))) {
      _Climber.setLeftClimbMotor(leftDemand);
    } else {
      _Climber.stopLeftClimbMotor();
    }

    // Right Climber
    double rightDemand = -0.5 * _RightSupplier.getAsDouble();
    if ((Math.abs(rightDemand) > Constants.minThumbstickMagnitude) && 
        ((rightDemand > 0 && !_Climber.rightAtMax()) || (rightDemand < 0 && !_Climber.rightAtHome()))) {
      _Climber.setRightClimbMotor(rightDemand);
    } else {
      _Climber.stopRightClimbMotor();
    }
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
