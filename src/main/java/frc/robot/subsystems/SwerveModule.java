package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Configs;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final SparkMax turnMotor;

    private final AbsoluteEncoder turnEncoder;
    private final SparkClosedLoopController turnController;

    private double angleOffset;

    public SwerveModule(int driveId, int turnId, double angleOffset) {
        driveMotor = new TalonFX(driveId);
        turnMotor = new SparkMax(turnId, MotorType.kBrushless);

        turnEncoder = turnMotor.getAbsoluteEncoder();
        turnController = turnMotor.getClosedLoopController();

        driveMotor.getConfigurator().apply(Configs.KrakenSwerveModule.driveConfig);

        turnMotor.configure(Configs.MAXSwerveModule.turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

        driveMotor.set(correctedDesiredState.speedMetersPerSecond / Constants.Swerve.MAX_LINEAR_SPEED);
        turnController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    }

}
