// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }

  public static class PneumaticsConstants {
    public static final int kAmpArmChannel = 10;
    public static final int kLeftBrakeChannel = 9;
    public static final int kRightBrakeChannel = 8;
  }

  public static class CANbusIds {
    public static final int kLeftClimbMotorID = 13;
    public static final int kRightClimbMotorID = 14;
    public static final int kIntakeMotorID = 10;
    public static final int kPneumaticsModuleID = 16;
    public static final int kLeftFlyWheelID = 11;
    public static final int kRightFlyWheelID = 12;
    public static final int FrontRightSteerMotorID = 8;
    public static final int FrontLeftSteerMotorID = 2;
    public static final int BackRightSteerMotorID = 6;
    public static final int BackLeftSteerMotorID = 4;
    public static final int FrontRightDriveMotorID = 9;
    public static final int FrontLeftDriveMotorID = 3;
    public static final int BackRightDriveMotorID = 7;
    public static final int BackLeftDriveMotorID = 5;   
    public static final int PidgeonID = 17;
    public static final int PowerDistributionHub = 15;
  }

  public static class MotorSpeeds {
    public static final double kIntakeSpeed = 1.0;
    public static final double kAmpShootRPM = 500; // TODO come back check speed
    public static final double kSpeakerShootRPM = 1575; //TODO come back check speed
    public static final double kClimberSpeed = 0.5;
  }

  public static class DioChannels {
    public static final int kNoteSensorChannel = 0;
    public static final int kLeftHomeSwitch = 1;
    public static final int kRightHomeSwitch = 2;
  }

  public static class SoftLimits {
    public static final double kClimberMax = 147000;
  }

  
  // Calculations required for driving motor conversion factors and feed forward
  public static final int kDrivingMotorPinionTeeth = 14;
  public static final double NeoMotorFreeSpeedRpm = 5676;
  public static final double kDrivingMotorFreeSpeedRps = NeoMotorFreeSpeedRpm / 60;
  public static final double kWheelDiameterMeters = 0.0762;
  public static final double kWheelCircumfrenceMeters = kWheelDiameterMeters * Math.PI; 
  public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
  public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumfrenceMeters) / kDrivingMotorReduction; 

  public static final double kDrivingEncoderPositionFactor = kWheelCircumfrenceMeters / kDrivingMotorReduction;
  public static final double kDrivingEncoderVelocityFactor = kWheelCircumfrenceMeters / kDrivingMotorReduction / 60.0;
  
  public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
  public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

  // Angular offsets of the modules relative to the chassis in radians
  public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
  public static final double kFrontRightChassisAngularOffset = 0;
  public static final double kBackLeftChassisAngularOffset = Math.PI;
  public static final double kBackRightChassisAngularOffset = Math.PI / 2;

  // Chassis configuration
  public static final double kTrackWidth = Units.inchesToMeters(30.0 - (2*1.75)); // Wdith in Y-Axis direction (left right)
  public static final double kWheelBase = Units.inchesToMeters(26.0 - (2*1.75)); // Length in X-Axis direction (fwd back)
  public static final double kDriveBaseRadius = Math.sqrt(Math.pow(kTrackWidth/2.0,2) + Math.pow(kWheelBase/2.0,2)); // TODO check this math
  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(15.76);
  public static final double kMinSpeedMetersPerSecond = 0.01;
  public static final double kMaxAccelMetersPerSecondSquared = 2.0;
  public static final double kMaxAngularSpeed = 6.0; //kMaxSpeedMetersPerSecond / Units.inchesToMeters(18.55);
  public static final double minThumbstickMagnitude = 0.1;
  public static final Translation2d FrontRightLocation = new Translation2d(+kWheelBase / 2, -kTrackWidth / 2); // Positive X is fwd
  public static final Translation2d FrontLeftLocation = new Translation2d(+kWheelBase / 2, +kTrackWidth / 2); // Positive Y if left
  public static final Translation2d BackRightLocation = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
  public static final Translation2d BackLeftLocation = new Translation2d(-kWheelBase / 2, +kTrackWidth / 2);
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    FrontRightLocation,
    FrontLeftLocation,
    BackRightLocation,
    BackLeftLocation
  );
  public static final double panRateOfChangeLimit = 8.0; // Translation Drive Demand Rate-of-Change Limit, units/sec
  public static final double rotRateOfChangeLimit = 8.0; // Rotation Drive Demand Rate-of-Change Limit, units/sec
  public static final double teleAngleHoldFactor = 0.035; // Teleop vision targeting P-gain, 1/degrees

  // Drivetrain PID Parameters
  public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
  public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

  public static final double kDrivingP = 0.04;
  public static final double kDrivingI = 0;
  public static final double kDrivingD = 0;
  public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
  public static final double kDrivingMinOutput = -1;
  public static final double kDrivingMaxOutput = 1;

  public static final double kTurningP = 3;
  public static final double kTurningI = 0.001;
  public static final double kTurningD = 5;
  public static final double kTurningFF = 0;
  public static final double kTurningMinOutput = -1;
  public static final double kTurningMaxOutput = 1;

  public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
  public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

  public static final int kDrivingMotorCurrentLimit = 50; // amps
  public static final int kTurningMotorCurrentLimit = 20; // amps

  // Drivetrain low & high 'gear' speeds
  public static final double TurboSpeedMultiple = 0.90;
  public static final double HighSpeedMultiple = 1.0;
  public static final double LowSpeedMultiple = 0.50;

  // Flywheel PID Parameters
  public static final double kFlywheelFF = 1023.0/20450.0;
  public static final double kFlywheelP = 0.0;
  public static final double kFlywheelI = 0.0;
  public static final double kFlywheelD = 0.0;
  
  // AprilTag Number definitions
  public static class AprilTagIDs {
    public static final int NoTarget = 0;
    public static final int RedSpeaker = 4;
    public static final int RedAmp = 5;
    public static final int BlueAmp = 6;
    public static final int BlueSpeaker = 7;
  }
}
