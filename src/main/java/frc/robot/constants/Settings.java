package frc.robot.constants;

public interface Settings {
    public interface Drive {
        public double kP = 0, kI = 0, kD = 0;

        public double CURRENT_LIMIT = 40;

        public double DRIVE_REDUCTION = 0;
    }
}
