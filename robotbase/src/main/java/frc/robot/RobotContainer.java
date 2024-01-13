// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.SWERVE.MAX_SPEED_METERS_PER_SECOND * 0.1).withRotationalDeadband(Constants.SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // private final Telemetry logger = new Telemetry(); // This puts a TON of stuff on shuffleboard.

  private void configureBindings() {
    Robot.m_drive.setDefaultCommand( // Robot.m_drive will execute this command periodically
        Robot.m_drive.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * Constants.SWERVE.MAX_SPEED_METERS_PER_SECOND) 
        // Drive forward with negative Y (forward)
          .withVelocityY(-joystick.getLeftX() * Constants.SWERVE.MAX_SPEED_METERS_PER_SECOND) // Drive left with negative X (left)
          .withRotationalRate(-joystick.getRightX() * Constants.SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND) 
          // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(Robot.m_drive.applyRequest(() -> brake));
    joystick.b().whileTrue(Robot.m_drive
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(Robot.m_drive.runOnce(() -> Robot.m_drive.seedFieldRelative()));
    joystick.back().onTrue(new InstantCommand(() -> {Robot.m_drive.tareEverything();}));

    // Robot.m_drive.register Telemetry(logger::telemeterize); //Shuffleboard fanatic
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
