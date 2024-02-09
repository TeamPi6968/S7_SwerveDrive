// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.Swerve.Module;;

public class MAXSwerveModule {
  private final CANSparkMax m_drivingSparkMax;
  // private final TalonFX m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  public final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  public final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, CANSparkLowLevel.MotorType.kBrushless);
    // m_drivingSparkMax = new TalonFX(drivingCANId);
    m_turningSparkMax = new CANSparkMax(turningCANId, CANSparkLowLevel.MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(Module.Driving.kEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(Module.Driving.kEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(Module.Turning.kEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(Module.Turning.kEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(Module.Turning.kEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(Module.Turning.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(Module.Turning.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(Module.Driving.kP);
    m_drivingPIDController.setI(Module.Driving.kI);
    m_drivingPIDController.setD(Module.Driving.kD);
    m_drivingPIDController.setFF(Module.Driving.kFF);
    m_drivingPIDController.setOutputRange(Module.Driving.kMinOutput, Module.Driving.kMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    // m_turningPIDController.setP(ModuleConstants.kTurningP);
    // m_turningPIDController.setI(ModuleConstants.kTurningI);
    // m_turningPIDController.setD(ModuleConstants.kTurningD);
    // m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    // m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
    //     ModuleConstants.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(Module.Driving.kMotorIdleMode);
    m_turningSparkMax.setIdleMode(Module.Turning.kMotorIdleMode);

    m_drivingSparkMax.setSmartCurrentLimit(Module.Driving.kMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(Module.Turning.kMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public double getPositionTurningEncoder() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return (m_turningEncoder.getPosition() - m_chassisAngularOffset);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    public void SetPID(double P, double I, double D) {
      m_turningPIDController.setP(P);
      m_turningPIDController.setI(I);
      m_turningPIDController.setD(D);
      m_turningPIDController.setFF(Module.Turning.kFF);
      m_turningPIDController.setOutputRange(Module.Turning.kMinOutput, Module.Turning.kMaxOutput);
    }
}
