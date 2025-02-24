package frc.robot.constants;

public interface Ports {
    public interface Drive {
        public double kP = 0, kI = 0, kD = 0;

        public double driveId = 0, turnId = 0;

        public double currentLimit = 40;
    }
}
