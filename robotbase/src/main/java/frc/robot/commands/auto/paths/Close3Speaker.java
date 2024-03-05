package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoPivotSetAngle;
import frc.robot.commands.auto.AutoPivotStop;
import frc.robot.commands.auto.AutoShooterSetRPMAndFinish;
import frc.robot.commands.auto.AutoShooterStopRPM;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class Close3Speaker extends SequentialCommandGroup {
  public Close3Speaker() {
    super(
        // First Shot
        new AutoPivotSetAngle(60),
        new AutoShooterSetRPMAndFinish(3000),
        new ShooterWaitForRPM().withTimeout(0.5),
        new IntakeRun(0.75, false).withTimeout(1),

        // Run path
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new AutoPivotSetAngle(31),
                new DriveChoreoPath("Close3Speaker.1"),
                new WaitCommand(1),
                new AutoPivotSetAngle(34.5),
                new DriveChoreoPath("Close3Speaker.2"),
                new WaitCommand(1),
                new AutoPivotSetAngle(35),
                new DriveChoreoPath("Close3Speaker.3"),
                new WaitCommand(2)),
            new IntakeRun(0.75, true),
            new ShooterSetRPM(4000, true)),

        // Stop motors
        new AutoPivotStop(),
        new AutoShooterStopRPM());
  }

  @Override
  public String toString() {
    return "Close3Speaker";
  }
}
