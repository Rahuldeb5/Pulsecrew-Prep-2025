package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.constants.Ports;

// drive talonFx, turn neo

public class SwerveModule {
    private TalonFX driveMotor;
    private SparkMax turnMotor;

    private AnalogEncoder turnEncoder;
    private double angleOffset;

    private TalonFXConfiguration driveConfig;

    private SparkClosedLoopController turnController;

    // private SwerveDriveKinematics driveKinematics;
    // private SwerveDriveKinematics turnKinematics;


    public SwerveModule(int driveId, int turnMotorId, int turnEncoderId, double angleOffset) {
        driveMotor = new TalonFX(driveId);
        turnMotor = new SparkMax(turnEncoderId, MotorType.kBrushless);

        this.angleOffset = angleOffset;

        turnEncoder = new AnalogEncoder(turnEncoderId);

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0.kP = Ports.Drive.kP;
        driveConfig.Slot0.kI = Ports.Drive.kI;
        driveConfig.Slot0.kD = Ports.Drive.kD;

        driveConfig.CurrentLimits.StatorCurrentLimit = Ports.Drive.currentLimit;

        driveMotor.getConfigurator().apply(driveConfig);

        turnController = turnMotor.getClosedLoopController();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), new Rotation2d((turnEncoder.get() - angleOffset)));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), new Rotation2d((turnEncoder.get() - angleOffset)));
    }

    public void setState(SwerveModuleState desiredState) {
        // this.desiredState = desiredState;
    }
}
