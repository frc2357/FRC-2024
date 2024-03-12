package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoPivotSetAngle;
import frc.robot.commands.auto.AutoShooterSetRPMAndFinish;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class LeftClose1Speaker extends SequentialCommandGroup {
  public LeftClose1Speaker() {
    super(
        new AutoPivotSetAngle(60),
        new AutoShooterSetRPMAndFinish(3000),
        new ShooterWaitForRPM().withTimeout(0.5),
        new IntakeRun(0.75, true).withTimeout(1),
        new ShooterStop(),
        new DriveChoreoPath("LeftClose1Speaker.1", "LeftClose1Speaker.1", false, true));
  }

  @Override
  public String toString() {
    return "LeftClose1Speaker";
  }
}
