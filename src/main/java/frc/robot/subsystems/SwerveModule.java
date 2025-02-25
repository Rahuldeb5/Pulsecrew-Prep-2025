package frc.robot.subsystems;

import java.util.Set;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.constants.Ports;
import frc.robot.constants.Settings;

// drive talonFx, turn neo

public class SwerveModule {
    private final TalonFX driveMotor;
    private final SparkMax turnMotor;

    private final AbsoluteEncoder turnEncoder;
    private final SparkClosedLoopController turnController;

    private final TalonFXConfiguration driveConfig;

    private double angleOffset;
    private SwerveModuleState desiredState;

    public SwerveModule(int driveId, int turnMotorId, double angleOffset) {
        driveMotor = new TalonFX(driveId);
        turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);

        turnEncoder = turnMotor.getAbsoluteEncoder();

        driveConfig = new TalonFXConfiguration();

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0.kP = Settings.Drive.kP;
        driveConfig.Slot0.kI = Settings.Drive.kI;
        driveConfig.Slot0.kD = Settings.Drive.kD;
        driveConfig.Feedback.SensorToMechanismRatio = Settings.Drive.DRIVE_REDUCTION;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = Settings.Drive.CURRENT_LIMIT;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Settings.Drive.CURRENT_LIMIT;
        driveConfig.CurrentLimits.StatorCurrentLimit = Settings.Drive.CURRENT_LIMIT;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        driveMotor.getConfigurator().apply(driveConfig);

        turnController = turnMotor.getClosedLoopController();

        desiredState = new SwerveModuleState(0.0, new Rotation2d());

        this.angleOffset = angleOffset;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), new Rotation2d((turnEncoder.getPosition() - angleOffset)));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), new Rotation2d((turnEncoder.getPosition() - angleOffset)));
    }

    public void setState(SwerveModuleState desiredState) {
        SwerveModuleState newState = new SwerveModuleState(desiredState.speedMetersPerSecond, Rotation2d.fromRadians(desiredState.angle.getRadians()+angleOffset));

        newState.optimize(Rotation2d.fromRotations(turnEncoder.getPosition()));

        turnController.setReference(newState.angle.getRotations(), ControlType.kPosition);
        driveMotor.set(desiredState.speedMetersPerSecond);

        this.desiredState = desiredState;

        
    }
    public void boom() {
        ADIS16470_IMU x = new ADIS16470_IMU();
        x.getAngle(IMUAxis.kZ);
        var y = 0;
    }
}
