package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Settings;

// drive talonFx, turn neo

public class SwerveModule {
    private final TalonFX driveMotor;
    private final SparkMax turnMotor;

    private final AbsoluteEncoder turnEncoder;
    private final SparkClosedLoopController turnController;

    private final TalonFXConfiguration driveConfig;

    private final SparkMaxConfig turnConfig;

    private double angleOffset; 
    private SwerveModuleState desiredState;

    public SwerveModule(int driveId, int turnMotorId, double angleOffset) {
        driveMotor = new TalonFX(driveId);
        turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);

        turnEncoder = turnMotor.getAbsoluteEncoder();
        turnController = turnMotor.getClosedLoopController();

        driveConfig = new TalonFXConfiguration();

        turnConfig = new SparkMaxConfig();

        /***
         * Drive Config
         */
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

        /***
         * Turn Config
         */
        turnConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(((int)Settings.Turn.CURRENT_LIMIT));
        turnConfig.absoluteEncoder
            .inverted(Settings.Turn.INVERTED)
            .positionConversionFactor(2 * Math.PI)
            .velocityConversionFactor(2 * Math.PI / 60.0);
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(Settings.Turn.kP, Settings.Turn.kI, Settings.Turn.kD)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2* Math.PI);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        desiredState = new SwerveModuleState(0.0, new Rotation2d());

        this.angleOffset = angleOffset;
    }

    // public SwerveModuleState getState() {
    //     return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), new Rotation2d((turnEncoder.getPosition() - angleOffset)));
    // }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), new Rotation2d((turnEncoder.getPosition() - angleOffset)));
    }

    public void setState(SwerveModuleState desiredState) {
        SwerveModuleState newState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle.plus(Rotation2d.fromRadians(angleOffset)));

        newState.optimize(new Rotation2d(turnEncoder.getPosition()));

        turnController.setReference(newState.angle.getRotations(), ControlType.kPosition);
        driveMotor.set(desiredState.speedMetersPerSecond / Settings.Drive.MAX_RPM);

        this.desiredState = desiredState;       
    }
}
