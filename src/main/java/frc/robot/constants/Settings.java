package frc.robot.constants;

public interface Settings {
    public double WIDTH = 0;
    public double LENGTH = 0;

    public double MAX_SPEED = 0;
    public double MAX_ANGULAR_SPEED = 0;

    public interface Drive {
        public double kP = 0, kI = 0, kD = 0;

        public double CURRENT_LIMIT = 40;

        public double DRIVE_REDUCTION = 0;

        public double MAX_RPM = 6000;
    }

    public interface Turn {
        public double kP = 0, kI = 0, kD = 0;

        public double CURRENT_LIMIT = 40;

        public double DRIVE_REDUCTION = 0;

        public boolean INVERTED = true;
    }
}
