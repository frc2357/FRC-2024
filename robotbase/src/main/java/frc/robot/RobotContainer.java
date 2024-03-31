// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.TargetLockOnSpeaker;
import frc.robot.commands.drive.TranslateToGamepiece;
import frc.robot.commands.intake.IntakeFeedToShooter;
import frc.robot.commands.intake.IntakeNoteFromFloor;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.commands.shooter.DefaultShooter;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class RobotContainer {
  private AutoCommandChooser m_autoCommandChooser;

  public RobotContainer() {
    registerNamedAutoCommands();
    m_autoCommandChooser = new AutoCommandChooser();
    Robot.swerve.setDefaultCommand(new DefaultDrive());
    Robot.pivot.setDefaultCommand(new PivotHoldAngle(Constants.PIVOT.DEFAULT_PIVOT_ANGLE));
    Robot.shooter.setDefaultCommand(new DefaultShooter());
  }

  public void registerNamedAutoCommands() {
    NamedCommands.registerCommand("ShooterPreloadPreset", new ParallelCommandGroup(
        new ShooterSetRPM(4000),
        new PivotHoldAngle(38)));
    NamedCommands.registerCommand("TranslateToGamepiece", new TranslateToGamepiece(3));
    NamedCommands.registerCommand("IntakeNoteFromFloor", new IntakeNoteFromFloor());
    NamedCommands.registerCommand("TargetLockOnSpeaker", new TargetLockOnSpeaker(true));
    NamedCommands.registerCommand("ShooterWaitForRPM", new ShooterWaitForRPM());
    NamedCommands.registerCommand("VisionTargeting", new VisionTargeting());
    NamedCommands.registerCommand("IntakeFeedToShooter", new IntakeFeedToShooter().withTimeout(0.2));
    NamedCommands.registerCommand("AutoShoot", new SequentialCommandGroup(
      new ParallelCommandGroup(new TargetLockOnSpeaker(true), new ShooterWaitForRPM()).withTimeout(SWERVE.AUTO_TARGET_LOCK_TIMEOUT_SECONDS),
      new IntakeFeedToShooter().withTimeout(0.25)
    ));
  }

  public Command getAutonomousCommand() {
    return m_autoCommandChooser.getSelectedAutoCommand();
  }
}
