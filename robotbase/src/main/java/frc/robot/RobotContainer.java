// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.drive.TranslateToGamepiece;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.intake.Pickup;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.shooter.DefaultShooter;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class RobotContainer {
  private AutoCommandChooser m_autoCommandChooser;

  public RobotContainer() {
    m_autoCommandChooser = new AutoCommandChooser();
    Robot.swerve.setDefaultCommand(new DefaultDrive());
    Robot.pivot.setDefaultCommand(new PivotHoldAngle(Constants.PIVOT.DEFAULT_PIVOT_ANGLE));
    Robot.shooter.setDefaultCommand(new DefaultShooter());
  }

  public Command getAutonomousCommand() {
    return m_autoCommandChooser.getSelectedAutoCommand();
  }
}
