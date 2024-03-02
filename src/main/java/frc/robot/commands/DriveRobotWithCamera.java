// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AprilTagIDs;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveRobotWithCamera extends Command {
  /** Creates a new DriveRobot. */

  private final Drivetrain _drivetrain;
  
  private DoubleSupplier _xSupplier, _ySupplier, _rotSupplier;

  private SlewRateLimiter _xLimiter = new SlewRateLimiter(Constants.panRateOfChangeLimit);
  private SlewRateLimiter _yLimiter = new SlewRateLimiter(Constants.panRateOfChangeLimit);
  private SlewRateLimiter _rotLimiter = new SlewRateLimiter(Constants.rotRateOfChangeLimit);

  private Rotation2d rotationTarget = null; // Angle to maintain if driver is not trying to turn

  public DriveRobotWithCamera(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
    this._drivetrain = drivetrain;
    
    this._xSupplier = xSupplier;
    this._ySupplier = ySupplier;
    this._rotSupplier = rotSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get demands from driver thumbsticks
    double xRawDemand = -1 * _xSupplier.getAsDouble(); 
    double yRawDemand = -1 *_ySupplier.getAsDouble();
    double rotRawDemand = -1 * _rotSupplier.getAsDouble();

    // Check magitude of driver inputs
    double leftMagnatude = Math.sqrt(xRawDemand*xRawDemand + yRawDemand*yRawDemand);
    double rightMagnatude = Math.abs(rotRawDemand);

    // Check if using camera or driver is trying to move
    int targetID = _drivetrain.getTargetID();
    boolean isAiming =  targetID != AprilTagIDs.NoTarget;
    boolean isDriving = leftMagnatude > Constants.minThumbstickMagnitude;
    boolean isTurning = rightMagnatude > Constants.minThumbstickMagnitude;
    


    // If driver is not trying to turn, prevent rotation with controller
    if (isDriving && !isTurning) {
      // Just stopped turning, rotation target still not set
      if (rotationTarget == null) {
        // Set target to most recent heading
        rotationTarget = _drivetrain.getPose().getRotation();
      }

      // Calculate error between target and current heading
      Rotation2d error = rotationTarget.minus(_drivetrain.getPose().getRotation());

      // Pass turning command to modules
      rotRawDemand = error.getDegrees() * Constants.teleAngleHoldFactor;
    } else {
      // Stopped driving OR started turning, stop trying to hold heading
      rotationTarget = null;
    }



    // If camera is on, start checking for results
    if (isAiming) {
      // Get list of all apriltags camera sees
      LimelightTarget_Fiducial[] detections = LimelightHelpers.getLatestResults("limelight").targetingResults.targets_Fiducials;
      
      // Create variable to track which apriltag we are targeting
      LimelightTarget_Fiducial target = null;

      // Check all apriltags detected
      for (var det:detections) {
        if (targetID == det.fiducialID) {
          // Apriltag seen matches the one camera is looking for
          if (target == null || det.ta > target.ta) {
            // This is either the first time seeing this apriltag OR...
            // somehow the camera is seeing two with the same ID and the new one is closer to the robot
            target = det;
          }
        }
      }

      // Camera found apriltag
      if (target != null) {
        // Overwrite driver turning command if camera found apriltag
        rotRawDemand = -1 * Constants.teleAngleHoldFactor * target.tx;
      }
    }



    // Square resulting inputs (and keep signs)
    double xDemand = _xLimiter.calculate( xRawDemand * Math.abs(xRawDemand));
    double yDemand = _yLimiter.calculate( yRawDemand * Math.abs(yRawDemand));
    double rotDemand = _rotLimiter.calculate( rotRawDemand * Math.abs(rotRawDemand));

    // Only command the modules to move if the driver is trying to move
    if (isDriving || isTurning) {
      // Drive
      if (_drivetrain.getDriveRobotRelative()) {
        _drivetrain.driveRobotRelative(xDemand, yDemand, rotDemand);
  
      } else {
        _drivetrain.driveFieldRelative(xDemand, yDemand, rotDemand);
  
      }
    } else {
      // Otherwise, tell modules to stop
      _drivetrain.stop();
      rotationTarget = null;
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
