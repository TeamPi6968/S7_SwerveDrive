// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.ErrorCode;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  // private final CANSparkMax m_drivingSparkMax;
  private final TalonFX m_drivingTalonFx;
  private final CANSparkMax m_turningSparkMax;

  //private final ErrorCode m_drivingEncoder;
  public final AbsoluteEncoder m_turningEncoder;

  //private final SparkMaxPIDController m_drivingPIDController;
  public final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  
  /* Swerve Angle Motor Configurations */
  // SupplyCurrentLimitConfiguration DriveSupplyLimit = new SupplyCurrentLimitConfiguration(ModuleConstants.angleEnableCurrentLimit, 
  //                                                                                        ModuleConstants.angleContinuousCurrentLimit, 
  //                                                                                        ModuleConstants.anglePeakCurrentLimit, 
  //                                                                                        ModuleConstants.anglePeakCurrentDuration);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ModuleConstants.driveKS, ModuleConstants.driveKV, ModuleConstants.driveKA);

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean DriveInverted) {
    // m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_drivingTalonFx = new TalonFX(drivingCANId);
    m_turningSparkMax = new CANSparkMax(turningCANId, CANSparkLowLevel.MotorType.kBrushless);

    TalonFXConfigurator TalonConfigurator = m_drivingTalonFx.getConfigurator();
    TalonFXConfiguration TalonConfiguration = new TalonFXConfiguration();
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // m_drivingSparkMax.restoreFactoryDefaults();
    m_drivingTalonFx.getConfigurator().apply(new TalonFXConfiguration());
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    // m_drivingEncoder = m_drivingSparkMax.getEncoder();
    //m_drivingTalonFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,20);
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    // m_drivingPIDController = m_drivingSparkMax.getPIDController();
    
    m_turningPIDController = m_turningSparkMax.getPIDController();
    // m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    //m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    //m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);
    m_drivingTalonFx.setInverted(DriveInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    TalonConfiguration.Slot0.kP = ModuleConstants.kDrivingP;
    TalonConfiguration.Slot0.kI = ModuleConstants.kDrivingI;
    TalonConfiguration.Slot0.kD = ModuleConstants.kDrivingD;
    // m_drivingTalonFx.config_kP(0, ModuleConstants.kDrivingP);
    // m_drivingTalonFx.config_kI(0, ModuleConstants.kDrivingI);
    // m_drivingTalonFx.config_kD(0, ModuleConstants.kDrivingD);
    // m_drivingTalonFx.config_kF(0, ModuleConstants.kDrivingFF);
    
    // m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    // m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    // m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    //m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    // m_turningPIDController.setP(ModuleConstants.kTurningP);
    // m_turningPIDController.setI(ModuleConstants.kTurningI);
    // m_turningPIDController.setD(ModuleConstants.kTurningD);
    // m_turningPIDController.setFF(ModuleConstants.kTurningFF);
     m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,ModuleConstants.kTurningMaxOutput);

    // m_drivingTalonFx.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    //m_drivingTalonFx.configSupplyCurrentLimit(DriveSupplyLimit);
    //m_drivingTalonFx.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    // m_drivingTalonFx.burnFlash();
    m_turningSparkMax.burnFlash();
    m_drivingTalonFx.getConfigurator().apply(motorConfigs);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingTalonFx.setPosition(0);
    // m_drivingTalonFx.setSelectedSensorPosition(0);
    // m_drivingEncoder.setPosition(0);
    
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingTalonFx.getSelectedSensorVelocity(),
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
    //Measure<Double> test = m_drivingTalonFx.getPosition();

    StatusSignal<Double> CurrentPosition = m_drivingTalonFx.getPosition();
    
    return new SwerveModulePosition(
       m_drivingTalonFx.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));//m_drivingTalonFx.getSelectedSensorPosition()
  }

  public double getPositionTurningEncoder() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return (m_turningEncoder.getPosition() - m_chassisAngularOffset);
  }

  public static double RPMToFalcon(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (2048.0 / 600.0);
    return sensorCounts;
}

  public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
}

  public static double MPS_Velocity(double meterpersecond){
    double velocity = 12 * (meterpersecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    return velocity;
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
   // m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

    // double velocity = MPSToFalcon(desiredState.speedMetersPerSecond, ModuleConstants.kWheelCircumferenceMeters, ModuleConstants.kDrivingMotorReduction);
    double velocity = MPS_Velocity(optimizedDesiredState.speedMetersPerSecond);
    m_drivingTalonFx.setControl( velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(optimizedDesiredState.speedMetersPerSecond));

    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalonFx.setPosition(0);
    //m_drivingEncoder.setPosition(0);
  }

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    public void SetPID(double P, double I, double D) {
      m_turningPIDController.setP(P);
      m_turningPIDController.setI(I);
      m_turningPIDController.setD(D);
      m_turningPIDController.setFF(ModuleConstants.kTurningFF);
      m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);
    }
}
