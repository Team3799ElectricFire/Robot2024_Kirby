// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.subsystems.Climber;

public class ClimberBothDown extends Command {
  private final Climber _Climber;
  /** Creates a new climArmDown. */
  public ClimberBothDown(Climber climber) {
    _Climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //_Climber.rightBrakeOff();
    //_Climber.leftBrakeOff();
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     
    if (!_Climber.leftAtHome()) {
      _Climber.setLeftClimbMotor(-MotorSpeeds.kClimberSpeed);
    } else {
      _Climber.stopLeftClimbMotor();
      _Climber.zeroLeftEncoder();
    }

    if (!_Climber.rightAtHome()){
      _Climber.setRightClimbMotor(-MotorSpeeds.kClimberSpeed);
    } else {
      _Climber.stopRightClimbMotor();
      _Climber.zeroRightEncoder();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (_Climber.leftAtHome()){
      _Climber.zeroLeftEncoder();
    }

    if (_Climber.rightAtHome()){
      _Climber.zeroRightEncoder();
    }
    
    _Climber.stopRightClimbMotor();
    _Climber.stopLeftClimbMotor();

    //_Climber.leftBrakeOn();
    //_Climber.rightBrakeOn();
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _Climber.leftAtHome() && _Climber.rightAtHome();
  }
}
