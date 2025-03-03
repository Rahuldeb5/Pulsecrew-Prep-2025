// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public interface OperatorConstants {
    int kDriverControllerPort = 0;
    double kDriveDeadband = 0.05;
  }

  public interface Swerve {
    double WIDTH = 18.75;
    double LENGTH = 18.75;

    SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.Swerve.LENGTH / 2, Constants.Swerve.WIDTH / 2),
      new Translation2d(Constants.Swerve.LENGTH / 2, -Constants.Swerve.WIDTH / 2),
      new Translation2d(-Constants.Swerve.LENGTH / 2, Constants.Swerve.WIDTH / 2),
      new Translation2d(-Constants.Swerve.LENGTH / 2, -Constants.Swerve.WIDTH / 2)
    );

    double MAX_LINEAR_SPEED = 4.0;
    double MAX_ANGULAR_SPEED = 2 * Math.PI;

    boolean FIELD_RELATIVE = true;

    public interface Drive {
      double kP = 0.0;
      double kI = 0.0;
      double kD = 0.0;
      double kS = 0.0;
      double kV = 0.0;
      double kA = 0.0;

      double motionMagicCruiseVelocity = 0.0;
      double motionMagicAcceleration = 0.0;
      double motionMagicJerk = 0.0;
    }

    public interface Turn {
      double kP = 0.0;
      double kI = 0.0;
      double kD = 0.0;
      double kS = 0.0;
      double kV = 0.0;
      double kA = 0.0;
    }

    double driveCurrentLimitAmps = 40;
    double turnCurrentLimitAmps = 20;

    double GEAR_RATIO = 5.36;
    double TURN_FACTOR = 2 * Math.PI;

    public interface FrontLeft {
      int TURN_ID = 15;
      int DRIVE_ID = 16;
      double ANGLE_OFFSET = Rotation2d.fromRotations(-0.149902).getRadians();
    }
    public interface FrontRight {
      int TURN_ID = 17;
      int DRIVE_ID = 10;
      double ANGLE_OFFSET = Rotation2d.fromRotations(-0.441162).getRadians();
    }
    public interface RearLeft {
      int TURN_ID = 13;
      int DRIVE_ID = 14;
      double ANGLE_OFFSET = Rotation2d.fromRotations(0.270752).getRadians();
    }
    public interface RearRight {
      int TURN_ID = 11;
      int DRIVE_ID = 12;
      double ANGLE_OFFSET = Rotation2d.fromRotations(0.113037).getRadians();
    }
  }

  public interface Auton {
    double MAX_LINEAR_SPEED = 3.0;
    double MAX_LINEAR_ACCELERATION_SQUARED = 3;
    double MAX_ANGULAR_SPEED = Math.PI;
    double MAX_ANGULAR_ACCELERATION_SQUARED = Math.PI;

    double kPX = 1;
    double kPY = 1;
    double kPTheta = 1;

    TrapezoidProfile.Constraints thetaControllerConstraints =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION_SQUARED);
  }
}
