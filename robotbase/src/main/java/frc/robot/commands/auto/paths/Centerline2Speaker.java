package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoPivotSetAngle;
import frc.robot.commands.auto.AutoPivotStop;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.auto.AutoShooterSetRPMAndFinish;
import frc.robot.commands.auto.AutoShooterStopRPM;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.intake.IntakeNoteFromFloor;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class Centerline2Speaker extends SequentialCommandGroup {
  public Centerline2Speaker() {
    super(
        // First Shot
        new AutoPivotSetAngle(60),
        new AutoShooterSetRPMAndFinish(3000),
        new ShooterWaitForRPM().withTimeout(0.5),
        new IntakeRun(0.75, true).withTimeout(1),
        new AutoShooterStopRPM(),

        // Run path
        new AutoPivotSetAngle(45),

        // get second note
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("RefSideFar2.1", true), new WaitCommand(1)),
            new IntakeNoteFromFloor()),

        // Preset RPM and Angle to be close to targe
        new AutoPivotSetAngle(50),
        new AutoShooterSetRPMAndFinish(4000),

        // Drive to spot to shoot second note
        new DriveChoreoPath("RefSideFar2.2"),

        // Shoot
        new AutoShoot(),
        new InstantCommand(
            () -> {
              System.out.println("Past auto shoot");
            }),

        // Go to next third note, and run intake
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                // get the third note
                new DriveChoreoPath("RefSideFar2.3"), new WaitCommand(1)),
            new IntakeNoteFromFloor()),

        // Preset RPM and Angle to be close to target
        new AutoShooterSetRPMAndFinish(4500),
        new AutoPivotSetAngle(50),

        // Drive to spot to shoot third note
        new DriveChoreoPath("RefSideFar2.4"),

        // shoot the third note
        new AutoShoot(),

        // Stop motors
        new AutoPivotStop(),
        new AutoShooterStopRPM());
  }

  @Override
  public String toString() {
    return getClass().getName();
  }
}
