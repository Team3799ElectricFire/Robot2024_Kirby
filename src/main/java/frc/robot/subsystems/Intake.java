// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DioChannels;
import frc.robot.Constants.CANbusIds;
import frc.robot.Constants.MotorSpeeds;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor; 
  private DigitalInput noteSensor;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(CANbusIds.kIntakeMotorID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.burnFlash();

    noteSensor = new DigitalInput(DioChannels.kNoteSensorChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Have Note", haveNote());
  }

  public void runIntake() {
    intakeMotor.set(MotorSpeeds.kIntakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public boolean haveNote() {
    return !noteSensor.get();
  }
}
