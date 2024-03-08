// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.Constants.MotorSpeeds;

public class ClimberRightDown extends Command {
  private final Climber _Climber;
  /** Creates a new ClimberLeftUp. */
  public ClimberRightDown(Climber climber) {
    _Climber= climber;
    addRequirements(climber);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //_Climber.rightBrakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_Climber.rightAtHome()){
      _Climber.stopRightClimbMotor();
      _Climber.zeroRightEncoder();

    } else {            
      _Climber.setRightClimbMotor(-1.0 * MotorSpeeds.kClimberSpeed);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (_Climber.rightAtHome()){
      _Climber.zeroRightEncoder();
    }

    _Climber.stopRightClimbMotor();

    //_Climber.rightBrakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _Climber.rightAtHome();
  }
}
