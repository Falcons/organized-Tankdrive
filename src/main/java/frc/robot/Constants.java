package frc.robot;

public class Constants {
    
    public static final class DriveConstants {

        // chassis constants
        public static final double trackWidthMeters = 0.5;
        public static final double wheelDiameterMeters = 0.102;
        public static final double ticksPerRotation = 40;
        // max MPS
        public static final double maxSpeedMPS = 5;

        // motor CANIDs:
        public static final int frontLeftCANID = 3;
        public static final int frontRightCANID = 9;
        public static final int backLeftCANID = 4;
        public static final int backRightCANID = 8;

        public static final int pigeonCANID = 6;
    }
    public static final class ControllerConstants {
        public static final double kDriveDeadband = 0.05;
    }
}
