// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VISION_PIPELINES;
import frc.robot.commands.drive.DefaultDrive;

public class RobotContainer {
  private AutoCommandChooser m_autoCommandChooser;

  public RobotContainer() {
    m_autoCommandChooser = new AutoCommandChooser();
    Robot.swerve.setDefaultCommand(new DefaultDrive());
    //Robot.pivot.setDefaultCommand(new DefaultPivot());
    Robot.shooterCam.setPipeline(VISION_PIPELINES.POSE_ESTIMATION_PIPELINE);
  }

  public Command getAutonomousCommand() {
    return m_autoCommandChooser.getSelectedAutoCommand();
  }
}
