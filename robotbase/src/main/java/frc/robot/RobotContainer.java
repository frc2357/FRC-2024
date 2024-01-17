// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.SWERVE.MAX_SPEED_METERS_PER_SECOND * 0.1)
          .withRotationalDeadband(
              SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private AutoCommandChooser m_autoCommandChooser;

  public RobotContainer() {
    configureBindings();
    m_autoCommandChooser = new AutoCommandChooser();
  }

  // private final Telemetry logger = new Telemetry(); // This puts a TON of stuff on shuffleboard.

  private void configureBindings() {
    Robot.drive.setDefaultCommand(
        Robot.drive.applyRequestCommand(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * Constants.SWERVE.MAX_SPEED_METERS_PER_SECOND)
                    // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX()
                            * Constants.SWERVE
                                .MAX_SPEED_METERS_PER_SECOND) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * Constants.SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND)
            // Drive counterclockwise with negative X (left)
            ));

    joystick.a().whileTrue(Robot.drive.applyRequestCommand(() -> brake));
    joystick
        .b()
        .whileTrue(
            Robot.drive.applyRequestCommand(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(Robot.drive.runOnce(() -> Robot.drive.seedFieldRelative()));
    joystick
        .back()
        .onTrue(
            new InstantCommand(
                () -> {
                  Robot.drive.tareEverything();
                }));

    // Robot.drive.register Telemetry(logger::telemeterize); //Shuffleboard fanatic
  }

  public Command getAutonomousCommand() {
    return m_autoCommandChooser.getSelectedAutoCommand();
  }
}
