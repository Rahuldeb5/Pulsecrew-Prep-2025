package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Settings;

public class SwerveDrive extends SubsystemBase {
    
    private final SwerveModule frontLeft, frontRight, rearLeft, rearRight;

    private final ADIS16470_IMU gyro;

    private final SwerveDriveKinematics driveKinematics;

    private SwerveDriveOdometry odometry;

    public SwerveDrive() {
        frontLeft = new SwerveModule(Ports.Drive.FrontLeft.driveId, Ports.Drive.FrontLeft.turnId, 0);
        frontRight = new SwerveModule(Ports.Drive.FrontRight.driveId, Ports.Drive.FrontRight.turnId, 0);
        rearLeft = new SwerveModule(Ports.Drive.RearLeft.driveId, Ports.Drive.RearLeft.turnId, 0);
        rearRight = new SwerveModule(Ports.Drive.RearRight.driveId, Ports.Drive.RearRight.turnId, 0);

        gyro = new ADIS16470_IMU();

        driveKinematics = new SwerveDriveKinematics(
            new Translation2d(Settings.LENGTH / 2, Settings.WIDTH / 2),
            new Translation2d(Settings.LENGTH / 2, -Settings.WIDTH / 2),
            new Translation2d(-Settings.LENGTH / 2, Settings.WIDTH / 2),
            new Translation2d(-Settings.LENGTH / 2, -Settings.WIDTH / 2));

        odometry = new SwerveDriveOdometry(
            driveKinematics,
            Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            });
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
        var swerveModuleStates = driveKinematics.toSwerveModuleStates(
            fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed*Settings.MAX_SPEED, ySpeed*Settings.MAX_SPEED, rot*Settings.MAX_ANGULAR_SPEED,
                Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ)))
                : new ChassisSpeeds(xSpeed*Settings.MAX_SPEED, ySpeed*Settings.MAX_SPEED, rot*Settings.MAX_ANGULAR_SPEED)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Settings.MAX_SPEED);
        
        frontLeft.setState(swerveModuleStates[0]);
        frontRight.setState(swerveModuleStates[1]);
        rearLeft.setState(swerveModuleStates[2]);
        rearRight.setState(swerveModuleStates[3]);
    }

    public void setX() {
        frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Settings.MAX_SPEED);

        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        rearLeft.setState(desiredStates[2]);
        rearRight.setState(desiredStates[3]);
    }
}
