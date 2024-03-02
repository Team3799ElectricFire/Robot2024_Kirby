// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveRobot extends Command {
  /** Creates a new DriveRobot. */

  private final Drivetrain _drivetrain;
  private DoubleSupplier _xSupplier, _ySupplier, _rotSupplier;
  private SlewRateLimiter _xLimiter = new SlewRateLimiter(8.0);
  private SlewRateLimiter _yLimiter = new SlewRateLimiter(8.0);
  private SlewRateLimiter _rotLimiter = new SlewRateLimiter(8.0);

  public DriveRobot(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
    this._drivetrain = drivetrain;
    addRequirements(drivetrain);
    this._xSupplier = xSupplier;
    this._ySupplier = ySupplier;
    this._rotSupplier = rotSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xRawDemand = -1 * _xSupplier.getAsDouble(); 
    double yRawDemand = -1 *_ySupplier.getAsDouble();
    double rotRawDemand = -1 * _rotSupplier.getAsDouble();

    double xDemand = _xLimiter.calculate( xRawDemand * Math.abs(xRawDemand));
    double yDemand = _yLimiter.calculate( yRawDemand * Math.abs(yRawDemand));
    double rotDemand = _rotLimiter.calculate( rotRawDemand * Math.abs(rotRawDemand));

    //magnitude
     double leftMagnatude = Math.sqrt(xRawDemand*xRawDemand + yRawDemand*yRawDemand);
     double rightMagnatude = Math.abs(rotRawDemand);

    // Only command the modules to move if the driver input is far enough from center
    if (leftMagnatude > Constants.minThumbstickMagnitude || rightMagnatude > Constants.minThumbstickMagnitude) {
      // Drive
      if (_drivetrain.getDriveRobotRelative()) {
        _drivetrain.driveRobotRelative(xDemand, yDemand, rotDemand);
  
      } else {
        _drivetrain.driveFieldRelative(xDemand, yDemand, rotDemand);
  
      }
    } else {
      // Stop
      _drivetrain.stop();
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
