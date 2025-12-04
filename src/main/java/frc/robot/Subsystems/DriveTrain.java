package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class DriveTrain extends SubsystemBase {

    // CREATE COMPONENTS:
    // create motors
    private final SparkMax frontLeft = new SparkMax(DriveConstants.frontLeftCANID, MotorType.kBrushless);
    private final SparkMax frontRight = new SparkMax(DriveConstants.frontRightCANID, MotorType.kBrushless);
    private final SparkMax backLeft = new SparkMax(DriveConstants.backLeftCANID, MotorType.kBrushless);
    private final SparkMax backRight = new SparkMax(DriveConstants.backRightCANID, MotorType.kBrushless);

    // create encoders
    private final RelativeEncoder frontLeftEncoder = frontLeft.getEncoder();
    private final RelativeEncoder frontRightEncoder = frontRight.getEncoder();

    // CONFIGS:
    // create motor configs
    private final SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    private final SparkMaxConfig backLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig backRightConfig = new SparkMaxConfig();

    // OTHER STUFF
    private final Pigeon2 gyro = new Pigeon2(0);
    // track width is distance between left and right wheels
    private final DifferentialDriveOdometry odometry;
    private DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

    public DriveTrain() {
        
        // reset gyro
        gyro.setYaw(0);

        // motor configs
        backLeftConfig.follow(frontLeft);
        backRightConfig.follow(frontRight);
        frontLeftConfig.inverted(true);
        // encoder configs
        frontLeftConfig.encoder.
            positionConversionFactor(DriveConstants.wheelDiameterInch * Math.PI / DriveConstants.ticksPerRotation);
        frontRightConfig.encoder.
            positionConversionFactor(DriveConstants.wheelDiameterInch * Math.PI / DriveConstants.ticksPerRotation);
      
        // flash motors
        backLeft.configure(backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        backRight.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontRight.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // create odometry
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic loop
        odometry.update(gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
      // reset encoders
      frontLeftEncoder.setPosition(0);
      frontRightEncoder.setPosition(0);
      
      // reset position
      odometry.resetPosition(
          gyro.getRotation2d(),
          frontLeftEncoder.getPosition(),
          frontRightEncoder.getPosition(),
          pose
      );
    }

    public void drive(double speed, double turn) {
        drive.arcadeDrive(speed, turn);
    }
}