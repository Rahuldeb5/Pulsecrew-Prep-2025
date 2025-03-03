package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeft, frontRight, rearLeft, rearRight;

    private final ADIS16470_IMU gyro;

    private SwerveDriveOdometry odometry;

    /***
     * Options for swerve modules:
     * Kraken (drive) + MAX (turn)
     * Kraken (drive + turn)
     * MAX (drive + turn)
     */

    public SwerveDrive() {
        frontLeft = new SwerveModule(Constants.Swerve.FrontLeft.DRIVE_ID, Constants.Swerve.FrontLeft.TURN_ID, Constants.Swerve.FrontLeft.ANGLE_OFFSET);
        frontRight = new SwerveModule(Constants.Swerve.FrontRight.DRIVE_ID, Constants.Swerve.FrontRight.TURN_ID, Constants.Swerve.FrontRight.ANGLE_OFFSET);
        rearLeft = new SwerveModule(Constants.Swerve.RearLeft.DRIVE_ID, Constants.Swerve.RearLeft.TURN_ID, Constants.Swerve.RearLeft.ANGLE_OFFSET);
        rearRight = new SwerveModule(Constants.Swerve.RearRight.DRIVE_ID, Constants.Swerve.RearRight.TURN_ID, Constants.Swerve.RearRight.ANGLE_OFFSET);

        gyro = new ADIS16470_IMU();

        odometry = new SwerveDriveOdometry(
            Constants.Swerve.driveKinematics,
            Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            });
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            },
            pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        double xSpeedDelivered = xSpeed * Constants.Swerve.MAX_LINEAR_SPEED;
        double ySpeedDelivered = ySpeed * Constants.Swerve.MAX_LINEAR_SPEED;
        double rotDelivered = rot * Constants.Swerve.MAX_ANGULAR_SPEED;
        
        var swerveModuleStates = Constants.Swerve.driveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        );
        
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }    

    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_LINEAR_SPEED);  

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            });
    }
}
