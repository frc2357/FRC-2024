// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.controls.CodriverControls;
import frc.robot.controls.DriverControls;

public class RobotContainer {
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.SWERVE.MAX_SPEED_METERS_PER_SECOND * 0.1)
          .withRotationalDeadband(
              SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  private AutoCommandChooser m_autoCommandChooser;

  public RobotContainer() {
    m_autoCommandChooser = new AutoCommandChooser();
    configureBindings();
  }

  private void configureBindings() {
    DriverControls driveControls =
        new DriverControls(
            new XboxController(Constants.CONTROLLER.DRIVE_CONTROLLER_PORT),
            Constants.CONTROLLER.DRIVE_CONTROLLER_DEADBAND);
    CodriverControls codriverControls =
        new CodriverControls(
            new XboxController(Constants.CONTROLLER.CODRIVER_CONTROLLER_PORT),
            Constants.CONTROLLER.CODRIVE_CONTROLLER_DEADBAND);

    Robot.drive.setDefaultCommand(
        Robot.drive.applyRequestCommand(
            () ->
                drive
                    .withVelocityX(driveControls.getX())
                    .withVelocityY(driveControls.getY())
                    .withRotationalRate(driveControls.getRotation())));
  }

  public Command getAutonomousCommand() {
    return m_autoCommandChooser.getSelectedAutoCommand();
  }
}
