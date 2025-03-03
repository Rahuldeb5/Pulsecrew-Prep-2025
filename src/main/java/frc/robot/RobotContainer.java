// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveDrive robotDrive = new SwerveDrive();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    robotDrive.setDefaultCommand(
      new RunCommand(
        () -> robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), Constants.OperatorConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), Constants.OperatorConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), Constants.OperatorConstants.kDriveDeadband), 
          Constants.Swerve.FIELD_RELATIVE),
        robotDrive)
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.rightTrigger()
      .whileTrue(new RunCommand(
        () -> robotDrive.setX(),
        robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig =
      new TrajectoryConfig(Constants.Auton.MAX_LINEAR_SPEED, Constants.Auton.MAX_LINEAR_ACCELERATION_SQUARED)
        .setKinematics(Constants.Swerve.driveKinematics);

    Trajectory mobilityAuton = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(0, -5), new Translation2d(0, 5)),
      new Pose2d(0, 0, new Rotation2d(0)),
      trajectoryConfig);
    
    ProfiledPIDController thetaController = new ProfiledPIDController(
      Constants.Auton.kPTheta, 0, 0,
      Constants.Auton.thetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI,Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      mobilityAuton,
      robotDrive::getPose, 
      Constants.Swerve.driveKinematics,
      
      new PIDController(Constants.Auton.kPX, 0, 0),
      new PIDController(Constants.Auton.kPY, 0, 0),
      thetaController,
      robotDrive::setModuleStates,
      robotDrive);

    robotDrive.resetOdometry(mobilityAuton.getInitialPose());

    return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
  }
}
