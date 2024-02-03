// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //-----------------------------------------------------------------------------------
  // SWerve Subsystem
  //-----------------------------------------------------------------------------------
  public static final class Swerve {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 0.5 * 4.8;// max 4.8
    public static final double kMaxAngularSpeed         = 0.25 * 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate  = 1.2; // radians per second
    public static final double kMagnitudeSlewRate  = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase  = Units.inchesToMeters(26.5);
    
    public static final SwerveDriveKinematics kDriveKinematics = 
      new SwerveDriveKinematics(new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset  = - Math.PI;
    public static final double kFrontRightChassisAngularOffset =   Math.PI;
    public static final double kBackLeftChassisAngularOffset   =   Math.PI;
    public static final double kBackRightChassisAngularOffset  = - Math.PI;

    public static final boolean kGyroReversed = false;

    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    //-----------------------------------------------------------------------------------
    // SwerveModule
    //-----------------------------------------------------------------------------------
    public static final class Module {

      public static final double kWheelDiameterMeters      = 0.0762; //[meters]
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

      //-----------------------------------------------------------------------------------
      // Driving
      //-----------------------------------------------------------------------------------
      public static final class Driving{
        // SPARK MAX CAN IDs
        public static final int kFrontLeftCanId  = 12;
        public static final int kRearLeftCanId   = 14;
        public static final int kFrontRightCanId = 10;
        public static final int kRearRightCanId  = 16;

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final int kDrivingMotorPinionTeeth = 14;
        public static final double kMotorReduction       = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);

        public static final double kFreeSpeedRpm      = 5676;
        public static final double kMotorFreeSpeedRps = kFreeSpeedRpm / 60;
        public static final double kWheelFreeSpeedRps = (kMotorFreeSpeedRps * kWheelCircumferenceMeters) / kMotorReduction;

        public static final double kEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kMotorReduction; // meters
        public static final double kEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kMotorReduction) / 60.0; // meters per second

        public static final double kP         = 0.04;
        public static final double kI         = 0;
        public static final double kD         = 0;
        public static final double kFF        = 1 / kWheelFreeSpeedRps;
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;

        public static final CANSparkBase.IdleMode kMotorIdleMode = CANSparkBase.IdleMode.kBrake;

        public static final int kMotorCurrentLimit = 50; // amps
      }

      //-----------------------------------------------------------------------------------
      // Turning
      //-----------------------------------------------------------------------------------
      public static final class Turning{
        // SPARK MAX CAN IDs
        public static final int kFrontLeftCanId  = 13;
        public static final int kRearLeftCanId   = 15;
        public static final int kFrontRightCanId = 11;
        public static final int kRearRightCanId  = 17;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kEncoderInverted      = true;
        public static final double kEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kEncoderPositionFactor; // radians

        public static final double kP         = 1;
        public static final double kI         = 0;
        public static final double kD         = 0;
        public static final double kFF        = 0;
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;

        public static final CANSparkBase.IdleMode kMotorIdleMode = CANSparkBase.IdleMode.kBrake;
    
        public static final int kMotorCurrentLimit = 20; // amps
    }
  }
    
  }

  public static final class Controller {
    //-----------------------------------------------------------------------------------
    // Controller
    //-----------------------------------------------------------------------------------
    public static final class Driver { 
      public static final int kControllerPort = 0;
      public static final double kDeadband    = 0.05;
    }

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond                = 0.5 * 3;
    public static final double kMaxAccelerationMetersPerSecondSquared  = 0.5 * 3;
    public static final double kMaxAngularSpeedRadiansPerSecond        = 0.5 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 0.5 * Math.PI;

    public static final double kPXController     = 1;
    public static final double kPYController     = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
      new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
