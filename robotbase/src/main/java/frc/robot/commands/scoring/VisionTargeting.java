package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.RobotMath;

public class VisionTargeting extends Command {

  public VisionTargeting() {
    addRequirements(Robot.pivot, Robot.shooter);
  }

  @Override
  public void execute() {
    boolean hasTarget = Robot.shooterCam.validTargetExists();
    if (hasTarget) {
      Double[] setpoints = calculateVisionTargetingSetpoints(Robot.shooterCam.getTY());
      SmartDashboard.putNumber("Current ty", Robot.shooterCam.getTY());
      if (!Double.isNaN(setpoints[0])) {
        SmartDashboard.putNumber("Pivot vision setpoint", setpoints[0]);
        Robot.pivot.setAngle(setpoints[0]);
      }
      if (!Double.isNaN(setpoints[1])) {
        SmartDashboard.putNumber("Shooter vision RPMs", setpoints[1]);
        Robot.shooter.setRPM(setpoints[1]);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.pivot.stop();
    Robot.shooter.stop();
  }

  private Double[] calculateVisionTargetingSetpoints(double ty) {
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, ty);
    if (curveIndex == -1) {
      // System.err.println("----- Curve segment index out of bounds (Pivot) -----");
      return new Double[] { Double.NaN, Double.NaN };
    }

    double[] high = Robot.shooterCurve[curveIndex];
    double[] low = Robot.shooterCurve[curveIndex + 1];

    double highTY = high[0];
    double lowTY = low[0];
    double highPivotRotation = high[1];
    double lowPivotRotation = low[1];
    double highShooterRPM = high[2];
    double lowShooterRPM = low[2];

    return new Double[] {
        RobotMath.linearlyInterpolate(highPivotRotation, lowPivotRotation, highTY, lowTY, ty),
        RobotMath.linearlyInterpolate(highShooterRPM, lowShooterRPM, highTY, lowTY, ty)
    };
  }
}
