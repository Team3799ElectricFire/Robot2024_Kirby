// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANbusIds;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.SoftLimits;

public class Shooter extends SubsystemBase {
  private TalonFX leftFlyWheelMotor,rightFlyWheelMotor;
  private Solenoid ampArmSolenoid;

  /** Creates a new Shooter. */
  public Shooter() {
    ampArmSolenoid = new Solenoid(CANbusIds.kPneumaticsModuleID, PneumaticsModuleType.REVPH, PneumaticsConstants.kAmpArmChannel);

    StatorCurrentLimitConfiguration currentLimits = new StatorCurrentLimitConfiguration();
    currentLimits.currentLimit = SoftLimits.kFlyWheelCurrentLimit;
    currentLimits.enable = true;

    rightFlyWheelMotor = new TalonFX(CANbusIds.kRightFlyWheelID);
    rightFlyWheelMotor.configFactoryDefault();
    rightFlyWheelMotor.setInverted(true);
    rightFlyWheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFlyWheelMotor.setNeutralMode(NeutralMode.Coast);
    rightFlyWheelMotor.config_kF(0, Constants.kFlywheelFF);
    rightFlyWheelMotor.config_kP(0, Constants.kFlywheelP);
    rightFlyWheelMotor.config_kI(0, Constants.kFlywheelI);
    rightFlyWheelMotor.config_kD(0, Constants.kFlywheelD);
    rightFlyWheelMotor.configStatorCurrentLimit(currentLimits);
    //rightFlyWheelMotor.configClosedloopRamp(10.0);

    leftFlyWheelMotor = new TalonFX(CANbusIds.kLeftFlyWheelID);
    leftFlyWheelMotor.configFactoryDefault();
    leftFlyWheelMotor.follow(rightFlyWheelMotor);
    leftFlyWheelMotor.setInverted(InvertType.OpposeMaster);
    leftFlyWheelMotor.setNeutralMode(NeutralMode.Coast);
    leftFlyWheelMotor.configStatorCurrentLimit(currentLimits);
    //leftFlyWheelMotor.configClosedloopRamp(10.0);

    //SmartDashboard.putNumber("Amp RPM",650.0);
    //SmartDashboard.putNumber("Speaker Speed", 0.60);
    //SmartDashboard.putNumber("Source Speed", 0.20);
    //SmartDashboard.putBoolean("Use Shelf", true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Flywheel Speed (rpm)", getFlywheelRpm());
  }

  public void extendArm() {
    //boolean useShelf = SmartDashboard.getBoolean("Use Shelf", false);
    ampArmSolenoid.set(true);
  }

  public void retractArm() {
    ampArmSolenoid.set(false);
  }

  public void prepShootAtAmp() {
    //double speed = SmartDashboard.getNumber("Amp RPM", 650.0);
    //rightFlyWheelMotor.set(TalonFXControlMode.Velocity, speed * 2048.0/600.0);
    rightFlyWheelMotor.set(TalonFXControlMode.Velocity, MotorSpeeds.kAmpShootRPM * 2048.0/600.0); 
  }

  public void prepShootAtSpeaker() {
    //double speed = SmartDashboard.getNumber("Speaker Speed", 0.60);
    //rightFlyWheelMotor.set(TalonFXControlMode.PercentOutput, speed);
    rightFlyWheelMotor.set(TalonFXControlMode.Velocity, MotorSpeeds.kSpeakerShootRPM * 2048.0/600.0); 
  }

  public void supplyFromHumanStation() {
    //double speed = SmartDashboard.getNumber("Source Speed", 0.20);
    //rightFlyWheelMotor.set(TalonFXControlMode.PercentOutput, -1.0*speed);
    rightFlyWheelMotor.set(TalonFXControlMode.Velocity, -1.0 * MotorSpeeds.kHumanFeederRPM * 2048.0/600.0);
  }

  public void shootStop() {
    rightFlyWheelMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public Command supplyFromHumanStationCommand() {
    return this.startEnd(this::supplyFromHumanStation, this::shootStop);
  }

  public double getFlywheelRpm() {
    return rightFlyWheelMotor.getSelectedSensorVelocity() * 600.0/2048.0;
  }
}
