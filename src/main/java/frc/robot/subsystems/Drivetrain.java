// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagIDs;
import frc.robot.Constants.CANbusIds;

public class Drivetrain extends SubsystemBase {
 // Swerve modules
  private SwerveModule FrontRightModule = new SwerveModule(
    CANbusIds.FrontRightDriveMotorID, 
    CANbusIds.FrontRightSteerMotorID, 
    Constants.kFrontRightChassisAngularOffset);
  private SwerveModule FrontLeftModule = new SwerveModule(
    CANbusIds.FrontLeftDriveMotorID, 
    CANbusIds.FrontLeftSteerMotorID, 
    Constants.kFrontLeftChassisAngularOffset);
  private SwerveModule BackRightModule = new SwerveModule(
    CANbusIds.BackRightDriveMotorID,
    CANbusIds.BackRightSteerMotorID,
    Constants.kBackRightChassisAngularOffset);
  private SwerveModule BackLeftModule = new SwerveModule(
    CANbusIds.BackLeftDriveMotorID,
    CANbusIds.BackLeftSteerMotorID,
    Constants.kBackLeftChassisAngularOffset);
 
  // Gyro sensor
  private WPI_Pigeon2 Pidgey = new WPI_Pigeon2(CANbusIds.PidgeonID);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry Odometry = new SwerveDriveOdometry(
    Constants.kDriveKinematics, 
    Pidgey.getRotation2d(),
    new SwerveModulePosition[] {
      FrontRightModule.getPosition(),
      FrontLeftModule.getPosition(),
      BackRightModule.getPosition(),
      BackLeftModule.getPosition()
    });

  private boolean _DriveRobotRelative = true;
  private double SpeedMultiple = Constants.LowSpeedMultiple;
  private Translation2d RotationCenter = new Translation2d();
  private int targetFiducialID = AprilTagIDs.NoTarget; // ID number of AprilTag we are aiming at (0 means no target, aka camera off)

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    //System.out.println("Track width:" + Constants.kTrackWidth);
    //System.out.println("Wheel base:" + Constants.kWheelBase);

    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPosePitch = 0.0;
    config.MountPoseRoll = -90.0;
    config.MountPoseYaw = 0.0;
    Pidgey.configAllSettings(config);

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    Constants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                    Constants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update odometry object tracking robot's position
    Odometry.update(
      Pidgey.getRotation2d(),
      new SwerveModulePosition[] {
        FrontRightModule.getPosition(),
        FrontLeftModule.getPosition(),
        BackRightModule.getPosition(),
        BackLeftModule.getPosition()
      }
    );

    // Update driverstation
    //UpdateDashboard();
    SmartDashboard.putNumber("Camera Target ID:", getTargetID());
  }
  
  public Pose2d getPose() {
    return Odometry.getPoseMeters();
  }
  public void resetPose(Pose2d pose) {
    Odometry.resetPosition(
      Pidgey.getRotation2d(),
      new SwerveModulePosition[] {
        FrontRightModule.getPosition(),
        FrontLeftModule.getPosition(),
        BackRightModule.getPosition(),
        BackLeftModule.getPosition()
      },
      pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.kDriveKinematics.toChassisSpeeds(getModuleState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a new variable
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);

    FrontRightModule.setDesiredState(moduleStates[0]);
    FrontLeftModule.setDesiredState(moduleStates[1]);
    BackRightModule.setDesiredState(moduleStates[2]);
    BackLeftModule.setDesiredState(moduleStates[3]);
  }
  public void driveRobotRelative(double xSpeed, double ySpeed, double rot) {
    double xVelocityMetersPerSecond = xSpeed * Constants.kMaxSpeedMetersPerSecond;
    double yVelocityMetersPerSecond = ySpeed * Constants.kMaxSpeedMetersPerSecond;
    double rotationRadiansPerSecond = rot * Constants.kMaxAngularSpeed;

    ChassisSpeeds speeds = new ChassisSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond, rotationRadiansPerSecond);

    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a new variable
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);

    FrontRightModule.setDesiredState(moduleStates[0]);
    FrontLeftModule.setDesiredState(moduleStates[1]);
    BackRightModule.setDesiredState(moduleStates[2]);
    BackLeftModule.setDesiredState(moduleStates[3]);
  }
  public void driveFieldRelative(double xSpeed, double ySpeed, double rot) {
    double xVelocityMetersPerSecond = xSpeed * Constants.kMaxSpeedMetersPerSecond;
    double yVelocityMetersPerSecond = ySpeed * Constants.kMaxSpeedMetersPerSecond;
    double rotationRadiansPerSecond = rot * Constants.kMaxAngularSpeed;

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond, rotationRadiansPerSecond, Pidgey.getRotation2d());
  
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a new variable
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);

    FrontRightModule.setDesiredState(moduleStates[0]);
    FrontLeftModule.setDesiredState(moduleStates[1]);
    BackRightModule.setDesiredState(moduleStates[2]);
    BackLeftModule.setDesiredState(moduleStates[3]);
  }
  public void setX() {
    FrontRightModule.setDesiredStateNoRestrictions(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    FrontLeftModule.setDesiredStateNoRestrictions(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    BackRightModule.setDesiredStateNoRestrictions(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    BackLeftModule.setDesiredStateNoRestrictions(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }
  public void stop() {
    FrontRightModule.stop();
    FrontLeftModule.stop();
    BackRightModule.stop();
    BackLeftModule.stop();
  }
  public SwerveModuleState[] getModuleState() {

    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = FrontRightModule.getState();
    states[1] = FrontLeftModule.getState();
    states[2] = BackRightModule.getState();
    states[3] = BackLeftModule.getState();

    return states;
  }
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);
    FrontRightModule.setDesiredState(desiredStates[0]);
    FrontLeftModule.setDesiredState(desiredStates[1]);
    BackRightModule.setDesiredState(desiredStates[2]);
    BackLeftModule.setDesiredState(desiredStates[3]);
  }
  public void setChassisSpeeds(ChassisSpeeds desiredSpeeds) {
    SmartDashboard.putNumber("Auto X", desiredSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Auto Y", desiredSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Auto Rot", desiredSpeeds.omegaRadiansPerSecond);

    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(desiredSpeeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a new variable
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);

    FrontRightModule.setDesiredState(moduleStates[0]);
    FrontLeftModule.setDesiredState(moduleStates[1]);
    BackRightModule.setDesiredState(moduleStates[2]);
    BackLeftModule.setDesiredState(moduleStates[3]);
  }

  public void resetEncoders() {
    FrontRightModule.resetEncoders();
    FrontLeftModule.resetEncoders();
    BackRightModule.resetEncoders();
    BackLeftModule.resetEncoders();
  }

  public void setHeading(double newAngleDegrees) {
    Pidgey.setYaw(newAngleDegrees);
  }
  public void zeroHeading() {
    Pidgey.reset();
  }
  public double getHeading() {
    return Pidgey.getRotation2d().getDegrees();
  }
  public double getTurnRate() {
    return Pidgey.getRate();
  }
  public double getPitch() {
    return Pidgey.getPitch();
  }
  public double getRoll() {
    return Pidgey.getRoll();
  }

  public void setDriveRobotRelative() {
    this._DriveRobotRelative = true;
    System.out.println("Set Robot Relative");
  }
  public void setDriveFieldRelative() {
    this._DriveRobotRelative = false;
    System.out.println("Set Field Relative");
  }
  public void toggleDriveRobotRelative() {
    this._DriveRobotRelative = !this._DriveRobotRelative;
  }
  public boolean getDriveRobotRelative() {
    return this._DriveRobotRelative;
  }
  public void setTurboSpeed() { // TODO add functions for "ShiftUP" and "ShiftDown"
    SpeedMultiple = Constants.TurboSpeedMultiple;
  }
  public void setHighSpeed() {
    SpeedMultiple = Constants.HighSpeedMultiple;
  }
  public void setLowSpeed() {
    SpeedMultiple = Constants.LowSpeedMultiple;
  }
  public void toggleHiLoSpeed() {
    if (SpeedMultiple == Constants.LowSpeedMultiple) {
      setHighSpeed();
    } else {
      setLowSpeed();
    }
  }
  public void setRotationCenterGamePiece(Translation2d newCenter) {
    RotationCenter = newCenter;
  }
  public void resetRotationCenterRobot() {
    RotationCenter = new Translation2d();
  }

  public void setTargetNone() {
     targetFiducialID = AprilTagIDs.NoTarget;
  }
  public void setTargetAmp() {
    //Check Alliance Color : https://docs.wpilib.org/en/stable/docs/software/basic-programming/alliancecolor.html
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        targetFiducialID = AprilTagIDs.RedAmp;
      }
      if (ally.get() == Alliance.Blue) {
        targetFiducialID = AprilTagIDs.BlueAmp;
      }
    } else {
      targetFiducialID = AprilTagIDs.NoTarget;
    }
  }
  public void setTargetSpeaker() {
    //Check Alliance Color: https://docs.wpilib.org/en/stable/docs/software/basic-programming/alliancecolor.html
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        targetFiducialID = AprilTagIDs.RedSpeaker;
      }
      if (ally.get() == Alliance.Blue) {
        targetFiducialID = AprilTagIDs.BlueSpeaker;
      }
    } else {
      targetFiducialID = AprilTagIDs.NoTarget;
    }
  }
  public int getTargetID() {
    return targetFiducialID;
  }
  public Command TargetAmpCommand() {
    return this.startEnd(this::setTargetAmp, this::setTargetNone).asProxy();
  }
  public Command TargetSpeakerCommand() {
    return this.startEnd(this::setTargetSpeaker, this::setTargetNone).asProxy();
  }

  public void UpdateDashboard() {
    // Report to driverstation
    SmartDashboard.putNumber("Drivetrain Gyro", Pidgey.getRotation2d().getDegrees());
    SmartDashboard.putNumber("FL Abs Angle", FrontLeftModule.getState().angle.getDegrees());
    SmartDashboard.putNumber("BL Abs Angle", BackLeftModule.getState().angle.getDegrees());
    SmartDashboard.putNumber("BR Abs Angle", BackRightModule.getState().angle.getDegrees());
    SmartDashboard.putNumber("FR Abs Angle", FrontRightModule.getState().angle.getDegrees());
    SmartDashboard.putNumber("Rot Velocity", Pidgey.getRate());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Speed Multiple", SpeedMultiple);
    SmartDashboard.putBoolean("Robot Relative Mode", _DriveRobotRelative);
  }
}
