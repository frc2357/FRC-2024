package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.RobotMath;

// TODO: This command will eventually run with PivotVisionTargeting
public class ShooterVisionTargeting extends Command {

  public ShooterVisionTargeting() {
    addRequirements(Robot.shooter);
  }

  @Override
  public void execute() {
    boolean hasTarget = Robot.shooterCam.validTargetExists();
    if (hasTarget) {
      double targetAngle = calculateVisionTargetingAngle(Robot.shooterCam.getTY());
      if (!Double.isNaN(targetAngle)) {
        Robot.pivot.setAngle(targetAngle);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stop();
  }

  private double calculateVisionTargetingAngle(double ty) {
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, ty);
    if (curveIndex == -1) {
      System.err.println("----- Curve segment index out of bounds (Shooter)-----");
      return Double.NaN;
    }

    double[] high = Robot.shooterCurve[curveIndex];
    double[] low = Robot.shooterCurve[curveIndex + 1];

    double highTY = high[0];
    double lowTY = low[0];
    double highRPM = high[2];
    double lowRPM = low[2];

    double shooterRPM = RobotMath.linearlyInterpolate(highRPM, lowRPM, highTY, lowTY, ty);

    if (Double.isNaN(shooterRPM)) {
      System.err.println("----- Invalid shooter values -----");
      return Double.NaN;
    }

    return shooterRPM;
  }
}
