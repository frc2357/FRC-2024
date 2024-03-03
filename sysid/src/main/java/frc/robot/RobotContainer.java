// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveProportional;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Robot.drive = new Drive();

    Robot.controller = new CommandXboxController(0);

    Robot.drive.setDefaultCommand(new DriveProportional());

    Robot.controller.a().whileTrue(Robot.drive.sysIdQuasistatic(Direction.kForward));
    Robot.controller.b().whileTrue(Robot.drive.sysIdQuasistatic(Direction.kReverse));
    Robot.controller.x().whileTrue(Robot.drive.sysIdQuasistatic(Direction.kForward));
    Robot.controller.y().whileTrue(Robot.drive.sysIdQuasistatic(Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }
}
