package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

// Based on REV documentation: https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java
public class SwerveModule {
    private CANSparkMax SteerMotor, DriveMotor;
    private AbsoluteEncoder SteerEncoder;
    private RelativeEncoder DriveEncoder;
    private SparkPIDController DrivePID, SteerPID;
    private double SteerOffset;
    private SwerveModuleState DesiredState = new SwerveModuleState(0.0, new Rotation2d());
    
    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        SteerMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);
        DriveMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);

        // Resetting
        //SteerMotor.restoreFactoryDefaults();
        DriveMotor.restoreFactoryDefaults();

        // Linking encoders and motors to PIDs
        DriveEncoder = DriveMotor.getEncoder();
        SteerEncoder = SteerMotor.getAbsoluteEncoder(Type.kDutyCycle); 
        DrivePID = DriveMotor.getPIDController();
        SteerPID = SteerMotor.getPIDController();
        DrivePID.setFeedbackDevice(DriveEncoder);
        SteerPID.setFeedbackDevice(SteerEncoder);

        // Apply converstion factors for encoders
        DriveEncoder.setPositionConversionFactor(Constants.kDrivingEncoderPositionFactor);
        DriveEncoder.setVelocityConversionFactor(Constants.kDrivingEncoderVelocityFactor);
        SteerEncoder.setPositionConversionFactor(Constants.kTurningEncoderPositionFactor);
        SteerEncoder.setVelocityConversionFactor(Constants.kTurningEncoderVelocityFactor);
        SteerEncoder.setInverted(true);

        // Enable steer wraparound
        SteerPID.setPositionPIDWrappingEnabled(true);
        SteerPID.setPositionPIDWrappingMinInput(Constants.kTurningEncoderPositionPIDMinInput);
        SteerPID.setPositionPIDWrappingMaxInput(Constants.kTurningEncoderPositionPIDMaxInput);

        // Set PID gains
        DrivePID.setP(Constants.kDrivingP);
        DrivePID.setI(Constants.kDrivingI);
        DrivePID.setD(Constants.kDrivingD);
        DrivePID.setFF(Constants.kDrivingFF);
        DrivePID.setOutputRange(Constants.kDrivingMinOutput, Constants.kDrivingMaxOutput);
        SteerPID.setP(Constants.kTurningP);
        SteerPID.setI(Constants.kTurningI);
        SteerPID.setD(Constants.kTurningD);
        SteerPID.setFF(Constants.kTurningFF);
        SteerPID.setOutputRange(Constants.kTurningMinOutput, Constants.kTurningMaxOutput);

        // Final motor setup
        SteerMotor.setIdleMode(Constants.kTurningMotorIdleMode);
        DriveMotor.setIdleMode(Constants.kDrivingMotorIdleMode);
        DriveMotor.setSmartCurrentLimit(Constants.kDrivingMotorCurrentLimit);
        SteerMotor.setSmartCurrentLimit(Constants.kTurningMotorCurrentLimit);

        // Save SPARK MAX configuration
        DriveMotor.burnFlash();
        SteerMotor.burnFlash();

        SteerOffset = chassisAngularOffset;
        DesiredState.angle = new Rotation2d(SteerEncoder.getPosition());
        resetEncoders();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            DriveEncoder.getVelocity(),
            new Rotation2d(SteerEncoder.getPosition() - SteerOffset)
        );
    }

    public SwerveModuleState getDesiredState() {
        return DesiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            DriveEncoder.getPosition(),
            new Rotation2d(SteerEncoder.getPosition() - SteerOffset)
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Correct incoming desired state (which is relative to robot) with this modules offset angle to get a state relative to this modules mounting
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(SteerOffset));

        // Optimize desired state based on current angle (never rotate module more than 90degrees)
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState,
            new Rotation2d(SteerEncoder.getPosition())
        );

        // Always set the driving motor's speed
        DrivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        
        // But only set the steering motor's position if the driving motor is moving
        if (Math.abs(optimizedDesiredState.speedMetersPerSecond) > Constants.kMinSpeedMetersPerSecond) {
            SteerPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition); 
        }
        
        // Update last recorded desired state
        this.DesiredState = desiredState;
    }

    public void setDesiredStateNoRestrictions(SwerveModuleState desiredState) {
        // Correct incoming desired state (which is relative to robot) with this modules offset angle to get a state relative to this modules mounting
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(SteerOffset));

        // Optimize desired state based on current angle (never rotate module more than 90degrees)
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState,
            new Rotation2d(SteerEncoder.getPosition())
        );
        
        // Always set the driving motor's speed
        DrivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        
        // Always set the steering motor's position
        SteerPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
        
        // Update last recorded desired state
        this.DesiredState = desiredState;
    }

    public void stop() {
        DriveMotor.set(0.0);
        SteerMotor.set(0.0);
    }

    public void resetEncoders() {
        DriveEncoder.setPosition(0);
    }
}
