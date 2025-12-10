// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Commands.DriveSim;
import frc.robot.Subsystems.DriveTrainSim;

public class RobotContainer {

  // Xbox contorller
   private final XboxController controller = new XboxController(0);
  // create subsystems
  // private final DriveTrain driveTrain = new DriveTrain();
  private final DriveTrainSim driveTrainSim = new DriveTrainSim();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // make a default command
      driveTrainSim.setDefaultCommand(
        new DriveSim(
          driveTrainSim,
          () -> -MathUtil.applyDeadband(controller.getLeftY(), ControllerConstants.kDriveDeadband),
          () -> -MathUtil.applyDeadband(controller.getRightX(), ControllerConstants.kDriveDeadband)
      ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
