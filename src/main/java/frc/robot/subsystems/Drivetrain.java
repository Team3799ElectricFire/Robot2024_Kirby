// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    //System.out.println("Track width:" + Constants.kTrackWidth);
    //System.out.println("Wheel base:" + Constants.kWheelBase);
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
    UpdateDashboard();
  }
  
  public Pose2d getPose() {
    return Odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
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
  public void setRotationCenterGamePiece(Translation2d newCenter) {
    RotationCenter = newCenter;
  }
  public void resetRotationCenterRobot() {
    RotationCenter = new Translation2d();
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
