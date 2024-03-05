// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANbusIds;
import frc.robot.Constants.DioChannels;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.SoftLimits;

public class Climber extends SubsystemBase {
  private TalonFX leftClimbTalonSRX, rightClimbTalonSRX;
  private Solenoid leftBrakeSolenoid, rightBrakeSolenoid;
  private DigitalInput leftHomeSwitch, rightHomeSwitch;

  /** Creates a new Climber. */
  public Climber() {
    leftBrakeSolenoid = new Solenoid(CANbusIds.kPneumaticsModuleID, PneumaticsModuleType.REVPH,
        PneumaticsConstants.kLeftBrakeChannel);
    rightBrakeSolenoid = new Solenoid(CANbusIds.kPneumaticsModuleID, PneumaticsModuleType.REVPH,
        PneumaticsConstants.kRightBrakeChannel);

    leftHomeSwitch = new DigitalInput(DioChannels.kLeftHomeSwitch);
    rightHomeSwitch = new DigitalInput(DioChannels.kRightHomeSwitch);

    leftClimbTalonSRX = new TalonFX(CANbusIds.kLeftClimbMotorID);
    leftClimbTalonSRX.configFactoryDefault();
    leftClimbTalonSRX.setInverted(true);
    leftClimbTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    rightClimbTalonSRX = new TalonFX(CANbusIds.kRightClimbMotorID);
    rightClimbTalonSRX.configFactoryDefault();
    rightClimbTalonSRX.setInverted(true);
    rightClimbTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climb Encoder", getLeftEncoder());
    SmartDashboard.putNumber("Right Climb Encoder", getRightEncoder());
    SmartDashboard.putBoolean("Left Home Switch", leftAtHome());
    SmartDashboard.putBoolean("Right Home Switch", rightAtHome());
  }

  public void leftBrakeOn() {
    leftBrakeSolenoid.set(false);
  }

  public void leftBrakeOff() {
    leftBrakeSolenoid.set(true);
  }

  public void rightBrakeOn() {
    rightBrakeSolenoid.set(false);
  }

  public void rightBrakeOff() {
    rightBrakeSolenoid.set(true);
  }

  public boolean leftAtHome() {
    return !leftHomeSwitch.get();
  }

  public boolean rightAtHome() {
    return !rightHomeSwitch.get();
  }

  public void setLeftClimbMotor(double speed) {
    leftClimbTalonSRX.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setRightClimbMotor(double speed) {
    rightClimbTalonSRX.set(TalonFXControlMode.PercentOutput, speed);
  }

  public double getLeftEncoder() {
    return leftClimbTalonSRX.getSelectedSensorPosition();
  }

  public boolean leftAtMax() {
    return getLeftEncoder() > SoftLimits.kClimberMax;
  }

  public double getRightEncoder() {
    return rightClimbTalonSRX.getSelectedSensorPosition();
  }

  public boolean rightAtMax() {
    return getRightEncoder() > SoftLimits.kClimberMax;
  }

  public void stopLeftClimbMotor() {
    leftClimbTalonSRX.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void stopRightClimbMotor() {
    rightClimbTalonSRX.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void zeroLeftEncoder() {
    leftClimbTalonSRX.setSelectedSensorPosition(0.0);
  }

  public void zeroRightEncoder() {
    rightClimbTalonSRX.setSelectedSensorPosition(0.0);
  }
}
