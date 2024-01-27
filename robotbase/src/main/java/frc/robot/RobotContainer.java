// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.PIDTestCommand;
import frc.robot.util.Utility;
import frc.robot.commands.DefaultDriveCommand;

public class RobotContainer {
  private AutoCommandChooser m_autoCommandChooser;

  public RobotContainer() {
    m_autoCommandChooser = new AutoCommandChooser();
    Robot.drive.setDefaultCommand(new DefaultDriveCommand());
  }

  // private final Telemetry logger = new Telemetry(); // This puts a TON of stuff on shuffleboard.

  private void configureBindings() {
    Robot.drive.setDefaultCommand(new DefaultDriveCommand());
  }

  public Command getAutonomousCommand() {
    return m_autoCommandChooser.getSelectedAutoCommand();
  }
}
