// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Controller;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Fake controller
import frc.robot.subsystems.FakePS4Controller;
import frc.robot.subsystems.FakePS4Controller.Button;

// Normal controller
// import edu.wpi.first.wpilibj.PS4Controller;
// import edu.wpi.first.wpilibj.PS4Controller.Button;


//import java.util.ArrayList;
import java.util.List;

//import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  FakePS4Controller m_driverController = new FakePS4Controller(Controller.Driver.kControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(() -> m_robotDrive.drive(-MathUtil.applyDeadband(m_driverController.getLeftY(), Controller.Driver.kDeadband),
                                              -MathUtil.applyDeadband(m_driverController.getLeftX(), Controller.Driver.kDeadband),
                                              -MathUtil.applyDeadband(m_driverController.getRightX(), Controller.Driver.kDeadband),
                                true, 
                                    true),
                                              m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // List<PathPlannerTrajectory> AutoPath = PathPlanner.loadPathGroup("Path_2", 
    //                                                                  AutoConstants.kMaxSpeedMetersPerSecond, 
    //                                                                  AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    //new SequentialCommandGroup(new FollowPathWithEvents(null, AutoPath.get(0), DriveConstants.AUTO_EVENT_MAP));

    // Create config for trajectory (Add kinematics to ensure max speed is actually obeyed)
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                                   AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Swerve.kDriveKinematics);
        
    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // 2. Generate trajectory with a modified intermediate point
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Starting coordinate [0,0]
      new Pose2d(0, 0, new Rotation2d(0)),
      // Adjusted intermediate point [2,0]
      List.of(new Translation2d(2, 0)),
      // Ending pose [2,0]
      new Pose2d(2, 0, Rotation2d.fromDegrees(0)), 
      config);

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

    var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, 
                                                    AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
                                                                                  m_robotDrive::getPose, // Functional interface to feed supplier
                                                                                  Swerve.kDriveKinematics,
                                                                                  // Position controllers
                                                                                  xController,
                                                                                  yController,
                                                                                  thetaController,
                                                                                  m_robotDrive::setModuleStates,
                                                                                  m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // SmartDashboard
    SmartDashboard.putNumber("ErrorPosition_X", xController.getPositionError());
    SmartDashboard.putNumber("ErrorPosition_Y", yController.getPositionError());
    SmartDashboard.putNumber("ErrorPosition_theta", thetaController.getPositionError());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
