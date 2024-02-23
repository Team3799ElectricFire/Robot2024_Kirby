// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANbusIds;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.PneumaticsConstants;

public class Shooter extends SubsystemBase {
  private TalonFX leftFlyWheelMotor,rightFlyWheelMotor;
  private Solenoid ampArmSolenoid;

  /** Creates a new Shooter. */
  public Shooter() {
    ampArmSolenoid = new Solenoid(CANbusIds.kPneumaticsModuleID, PneumaticsModuleType.REVPH, PneumaticsConstants.kAmpArmChannel);

    rightFlyWheelMotor = new TalonFX(CANbusIds.kRightFlyWheelID);
    rightFlyWheelMotor.configFactoryDefault();
    rightFlyWheelMotor.setInverted(true);
    rightFlyWheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFlyWheelMotor.setNeutralMode(NeutralMode.Coast);
    rightFlyWheelMotor.config_kF(0, Constants.kFlywheelFF);
    rightFlyWheelMotor.config_kP(0, Constants.kFlywheelP);
    rightFlyWheelMotor.config_kI(0, Constants.kFlywheelI);
    rightFlyWheelMotor.config_kD(0, Constants.kFlywheelD);
    rightFlyWheelMotor.configClosedloopRamp(10.0);

    leftFlyWheelMotor = new TalonFX(CANbusIds.kLeftFlyWheelID);
    leftFlyWheelMotor.configFactoryDefault();
    leftFlyWheelMotor.follow(rightFlyWheelMotor);
    leftFlyWheelMotor.setInverted(InvertType.OpposeMaster);
    leftFlyWheelMotor.setNeutralMode(NeutralMode.Coast);
    leftFlyWheelMotor.configClosedloopRamp(10.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flywheel Speed (encoder units)", rightFlyWheelMotor.getSelectedSensorVelocity());
  }

  public void extendArm() {
    ampArmSolenoid.set(true);
  }

  public void retractArm() {
    ampArmSolenoid.set(false);
  }

  public void prepShootAtAmp() {
    rightFlyWheelMotor.set(TalonFXControlMode.PercentOutput, 0.22);
    //rightFlyWheelMotor.set(TalonFXControlMode.Velocity, MotorSpeeds.kAmpShootRPM * 2048.0/600.0); 
  }

  public void prepShootAtSpeaker() {
    rightFlyWheelMotor.set(TalonFXControlMode.PercentOutput, 0.5);
    //rightFlyWheelMotor.set(TalonFXControlMode.Velocity, MotorSpeeds.kSpeakerShootRPM * 2048.0/600.0); 
  }

  public void shootStop() {
    rightFlyWheelMotor.set(TalonFXControlMode.Velocity, 0);
  }
}
