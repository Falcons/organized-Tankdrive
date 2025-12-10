package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;

public class DriveTrainSim extends SubsystemBase {

    // CREATE COMPONENTS:
    // create motors
    private final SparkMax frontLeft = new SparkMax(DriveConstants.frontLeftCANID, MotorType.kBrushless);
    private final SparkMax frontRight = new SparkMax(DriveConstants.frontRightCANID, MotorType.kBrushless);
    // private final SparkMax backLeft = new SparkMax(DriveConstants.backLeftCANID, MotorType.kBrushless);
    // private final SparkMax backRight = new SparkMax(DriveConstants.backRightCANID, MotorType.kBrushless);

    private final SparkMaxSim frontLeftSim = new SparkMaxSim(frontLeft, DCMotor.getNEO(1));
    private final SparkMaxSim frontRightSim = new SparkMaxSim(frontRight, DCMotor.getNEO(1));
    // private final SparkMaxSim backLeftSim = new SparkMaxSim(backLeft, DCMotor.getNEO(1));
    // private final SparkMaxSim backRightSim = new SparkMaxSim(backRight, DCMotor.getNEO(1));

    // create encoders
    private final SparkRelativeEncoderSim frontLeftEncoderSim = frontLeftSim.getRelativeEncoderSim();
    private final SparkRelativeEncoderSim frontRightEncoderSim = frontRightSim.getRelativeEncoderSim();

    // OTHER STUFF
    private final Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonCANID);
    // track width is distance between left and right wheels
    private final DifferentialDriveOdometry odometry;

    // send this to shuffle board
    private double botPose[] = {0,0,0};

    public DriveTrainSim() {
        // reset gyro
        gyro.setYaw(0);
        // create odometry
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), frontLeftEncoderSim.getPosition(), frontRightEncoderSim.getPosition());
    }

    @Override
    public void simulationPeriodic(){
        // update odometry and send to logger
        odometry.update(gyro.getRotation2d(), frontLeftEncoderSim.getPosition(), frontRightEncoderSim.getPosition());
        // update bot pose
        botPose[0] = getPose().getX();
        botPose[1] = getPose().getY();
        botPose[2] = getPose().getRotation().getRadians();
        // send updated array to SmartDashboard
        SmartDashboard.putNumberArray("Field", botPose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
      // reset encoders
      frontLeftEncoderSim.setPosition(0);
      frontRightEncoderSim.setPosition(0);
      
      // reset position
      odometry.resetPosition(
          gyro.getRotation2d(),
          frontLeftEncoderSim.getPosition(),
          frontRightEncoderSim.getPosition(),
          pose
      );
    }

    public void drive(double speed, double turn) {
        double deltaTime = 0.02; // each loop is 0.02 seconds
        
        // Get left/right wheel speeds from arcade IK
        WheelSpeeds outputs = DifferentialDrive.arcadeDriveIK(
            speed, turn, false);
    
        // Convert wheel speeds to distances use delta time so it isnt too fast
        double leftDistance = outputs.left * deltaTime * DriveConstants.maxSpeedMPS;
        double rightDistance = outputs.right * deltaTime * DriveConstants.maxSpeedMPS;
    
        // Update encoder positions
        frontLeftEncoderSim.setPosition(frontLeftEncoderSim.getPosition() + leftDistance);
        frontRightEncoderSim.setPosition(frontRightEncoderSim.getPosition() + rightDistance);
    
        // Update gyro
        double addRotation = 
            (rightDistance - leftDistance) / DriveConstants.trackWidthMeters;
        gyro.setYaw(
            gyro.getYaw().getValueAsDouble() + Math.toDegrees(addRotation));
    }
}