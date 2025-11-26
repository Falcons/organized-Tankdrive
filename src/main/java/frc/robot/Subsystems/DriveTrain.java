// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  private SparkMax frontLeft = new SparkMax(DriveConstants.frontLeftCANID, MotorType.kBrushless);
  private SparkMax frontRight = new SparkMax(3, MotorType.kBrushless);
  private SparkMax backLeft = new SparkMax(2, MotorType.kBrushless);
  private SparkMax backRight = new SparkMax(1, MotorType.kBrushless);

  private SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
  private SparkMaxConfig frontRightConfig = new SparkMaxConfig();
  private SparkMaxConfig backLeftConfig = new SparkMaxConfig();
  private SparkMaxConfig backRightConfig = new SparkMaxConfig();

  private DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    backLeftConfig.
      follow(frontLeft);
    backRightConfig.
      follow(frontRight);
    frontLeftConfig.
      inverted(true);
    
    backLeft.configure(backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRight.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(Double speed, Double turn) {
    drive.arcadeDrive(speed, turn);
  }
}
