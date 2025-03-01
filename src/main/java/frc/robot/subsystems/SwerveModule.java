package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
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
import frc.robot.Constants;


public class SwerveModule {
    private final TalonFX driveMotor;
    private final SparkMax turnMotor;

    private final AbsoluteEncoder turnEncoder;
    private final SparkClosedLoopController turnController;
    private final SparkMaxConfig turnConfig;

    private final TalonFXConfiguration driveConfig;

    private SwerveModuleState desiredState;
    private double angleOffset;

    public SwerveModule(int driveId, int turnId, double angleOffset) {
        driveMotor = new TalonFX(driveId);
        turnMotor = new SparkMax(turnId, MotorType.kBrushless);

        turnEncoder = turnMotor.getAbsoluteEncoder();
        turnController = turnMotor.getClosedLoopController();

        driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = new Slot0Configs().withKP(Constants.Swerve.Drive.kP).withKI(Constants.Swerve.Drive.kI).withKD(Constants.Swerve.Drive.kD);
        driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.GEAR_RATIO;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.Swerve.driveCurrentLimitAmps;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Constants.Swerve.driveCurrentLimitAmps;
        driveConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.driveCurrentLimitAmps;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;
        driveMotor.getConfigurator().apply(driveConfig);

        turnConfig = new SparkMaxConfig();
        turnConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int)Constants.Swerve.driveCurrentLimitAmps);
        turnConfig.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(Constants.Swerve.TURN_FACTOR)
            .velocityConversionFactor(Constants.Swerve.TURN_FACTOR / 60.0);
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(Constants.Swerve.Turn.kP, Constants.Swerve.Turn.kI, Constants.Swerve.Turn.kD)
            .outputRange(-1,1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0,Constants.Swerve.TURN_FACTOR);

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        desiredState = new SwerveModuleState(0.0, new Rotation2d(turnEncoder.getPosition()));
        this.angleOffset = angleOffset;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), new Rotation2d(turnEncoder.getPosition() - angleOffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), new Rotation2d(turnEncoder.getPosition()-angleOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angleOffset));

        correctedDesiredState.optimize(new Rotation2d(turnEncoder.getPosition()));

        driveMotor.set(correctedDesiredState.speedMetersPerSecond / Constants.Swerve.MAX_RPM);
        turnController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
        
        this.desiredState = desiredState;
    }

}
