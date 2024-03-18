// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DioChannels;
import frc.robot.Constants.CANbusIds;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.SoftLimits;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor; 
  private DigitalInput noteSensor;
  private DigitalOutput Pin0, Pin1, Pin2;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(CANbusIds.kIntakeMotorID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.setSmartCurrentLimit(SoftLimits.kIntakeCurrentLimit);
    intakeMotor.burnFlash();

    noteSensor = new DigitalInput(DioChannels.kNoteSensorChannel);

    Pin0 = new DigitalOutput(DioChannels.LightsPin0);
    Pin1 = new DigitalOutput(DioChannels.LightsPin1);
    Pin2 = new DigitalOutput(DioChannels.LightsPin2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Have Note", haveNote());
  }

  public void runIntake() {
    intakeMotor.set(MotorSpeeds.kIntakeSpeed);
  }

  public void ejectIntake() {
    intakeMotor.set(-MotorSpeeds.kIntakeSpeed);
  }

  public Command ejectIntakeCommand() {
    return this.startEnd(this::ejectIntake, this::stopIntake);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public boolean haveNote() {
    return !noteSensor.get();
  }

  public boolean noNote() {
    return !haveNote();
  }

  public void setMode(int newMode) {
    if (newMode >= 0 && newMode <= 7) {
      Pin0.set((newMode & 0x01) == 0);
      Pin1.set((newMode & 0x02) == 0);
      Pin2.set((newMode & 0x04) == 0);
    }
  }
}
