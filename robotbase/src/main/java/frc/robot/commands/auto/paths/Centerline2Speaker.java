package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        new IntakeRun(0.75, false).withTimeout(1),
        new AutoShooterStopRPM(),
        // Run path
        new SequentialCommandGroup(
            new AutoPivotSetAngle(45),
            // get second note
            new IntakeNoteFromFloor(),
            new DriveChoreoPath("RefSideFar2.1"),
            new WaitCommand(0.5),
            // go to the spot to shoot and shoot
            new DriveChoreoPath("RefSideFar2.2"),
            new AutoPivotSetAngle(50),
            new AutoShooterSetRPMAndFinish(4000),
            new WaitCommand(1),
            new AutoShooterStopRPM(),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    // get the third note
                    new DriveChoreoPath("RefSideFar2.3"), new WaitCommand(0.5)),
                new IntakeNoteFromFloor()),
            // go the spot to shoot the last note
            new ParallelCommandGroup(
                new AutoPivotSetAngle(50), new DriveChoreoPath("RefSideFar2.4")),
            // shoot the last note
            new AutoShooterSetRPMAndFinish(4500).andThen(new ShooterWaitForRPM()),
            new IntakeRun(0.75, false)),

        // Stop motors
        new AutoPivotStop(),
        new AutoShooterStopRPM());
  }
}
