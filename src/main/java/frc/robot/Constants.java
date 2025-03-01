// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

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
    double MAX_RPM = 6000;
    double MAX_ANGULAR_SPEED = 2 * Math.PI;

    boolean FIELD_RELATIVE = true;

    public interface Drive {
      double kP = 0.0;
      double kI = 0.0;
      double kD = 0.0;
    }

    public interface Turn {
      double kP = 0.0;
      double kI = 0.0;
      double kD = 0.0;
    }

    double driveCurrentLimitAmps = 40;

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
}
