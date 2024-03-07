package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoPivotSetAngle;
import frc.robot.commands.auto.AutoShooterSetRPMAndFinish;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class LeftClose1Speaker extends SequentialCommandGroup {
  public LeftClose1Speaker() {
    super(
        new AutoPivotSetAngle(60),
        new AutoShooterSetRPMAndFinish(3000),
        new ShooterWaitForRPM().withTimeout(0.5),
        new IntakeRun(0.75, false).withTimeout(1),
        new DriveChoreoPath("LeftClose1Speaker")
    );
  }
}
