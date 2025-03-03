package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();
    
        static {
            turnConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int)Constants.Swerve.turnCurrentLimitAmps);
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
        }
    }

    public static final class KrakenSwerveModule {
        public static final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration turnConfig = new TalonFXConfiguration();

        static {
            driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            driveConfig.Slot0 = new Slot0Configs()
                .withKP(Constants.Swerve.Drive.kP)
                .withKI(Constants.Swerve.Drive.kI)
                .withKD(Constants.Swerve.Drive.kD)
                .withKS(Constants.Swerve.Drive.kS)
                .withKV(Constants.Swerve.Drive.kV)
                .withKA(Constants.Swerve.Drive.kA);
            driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.GEAR_RATIO;
            driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.Swerve.driveCurrentLimitAmps;
            driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Constants.Swerve.driveCurrentLimitAmps;
            driveConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.driveCurrentLimitAmps;
            driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;
            driveConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Swerve.Drive.motionMagicCruiseVelocity;
            driveConfig.MotionMagic.MotionMagicAcceleration = Constants.Swerve.Drive.motionMagicAcceleration;
            driveConfig.MotionMagic.MotionMagicJerk = Constants.Swerve.Drive.motionMagicJerk;
        }
    }
}
