package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;

public class Close3Speaker extends SequentialCommandGroup {
  public Close3Speaker() {
    super(
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveChoreoPath("Close3Speaker.1"),
                new WaitCommand(1),
                new DriveChoreoPath("Close3Speaker.2"),
                new WaitCommand(1),
                new DriveChoreoPath("Close3Speaker.3"),
                new WaitCommand(2)),
            
            new PivotHoldAngle(55, true),
            new IntakeRun(0.75),
            new ShooterSetRPM(4000, true)));
  }

  @Override
  public String toString() {
    return "Close3Speaker";
  }
}
