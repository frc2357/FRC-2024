// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< Updated upstream
import frc.robot.commands.auto.DefaultDriveCommand;
=======
import frc.robot.commands.drive.DefaultDriveCommand;
>>>>>>> Stashed changes

public class RobotContainer {
  private AutoCommandChooser m_autoCommandChooser;

  public RobotContainer() {
    m_autoCommandChooser = new AutoCommandChooser();
    Robot.drive.setDefaultCommand(new DefaultDriveCommand());
  }

  public Command getAutonomousCommand() {
    return m_autoCommandChooser.getSelectedAutoCommand();
  }
}
