package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.pivot.PivotHoldAngle;
import frc.robot.commands.shooter.ShooterSetRPM;

public class VisionlessShooting extends ParallelDeadlineGroup {
  public VisionlessShooting(double shooterRPM, double pivotAngle) {
    super(new ShooterSetRPM(shooterRPM), new PivotHoldAngle(pivotAngle, true, true));
  }
}
