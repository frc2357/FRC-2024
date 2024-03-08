package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoPivotSetAngle;
import frc.robot.commands.auto.AutoPivotStop;
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
            new SequentialCommandGroup(new DriveChoreoPath("RefSideFar2.1"), new WaitCommand(1)),
            new IntakeNoteFromFloor()),
        new WaitCommand(0.5), // Should be able to remove

        // go to the spot to shoot and shoot
        new AutoPivotSetAngle(50),
        new AutoShooterSetRPMAndFinish(4000),
        new DriveChoreoPath("RefSideFar2.2"),
        new ShooterWaitForRPM().withTimeout(0.5),
        new IntakeRun(0.75, true).withTimeout(1),
        new AutoShooterStopRPM(),

        // Go to next third note, and run intake
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                // get the third note
                new DriveChoreoPath("RefSideFar2.3"), new WaitCommand(1)),
            new IntakeNoteFromFloor()),

        // go the spot to shoot the last note
        new AutoShooterSetRPMAndFinish(4500),
        new AutoPivotSetAngle(50),
        new DriveChoreoPath("RefSideFar2.4"),
        // shoot the last note
        new ShooterWaitForRPM().withTimeout(0.5),
        new IntakeRun(0.75, true).withTimeout(1),

        // Stop motors
        new AutoPivotStop(),
        new AutoShooterStopRPM());
  }

  @Override
  public String toString() {
    return getClass().getName();
  }
}
