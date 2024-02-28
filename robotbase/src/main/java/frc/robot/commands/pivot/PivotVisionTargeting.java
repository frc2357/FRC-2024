package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.RobotMath;

// TODO: Make this part of a larger command that handles the speed too.
public class PivotVisionTargeting extends Command {
  public PivotVisionTargeting() {}

  @Override
  public boolean isFinished() {
    return false;
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
  public void end(boolean interrupted) {
    Robot.pivot.stop();
  }

  private double calculateVisionTargetingAngle(double ty) {
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, ty);
    if (curveIndex == -1) {
      // System.err.println("----- Curve segment index out of bounds (Pivot) -----");
      return Double.NaN;
    }

    double[] high = Robot.shooterCurve[curveIndex];
    double[] low = Robot.shooterCurve[curveIndex + 1];

    double highTY = high[0];
    double lowTY = low[0];
    double highPivotRotation = high[1];
    double lowPivotRotation = low[1];

    double pivotAngle =
        RobotMath.linearlyInterpolate(highPivotRotation, lowPivotRotation, highTY, lowTY, ty);

    return pivotAngle;
  }
}
