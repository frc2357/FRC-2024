package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.RobotMath;

public class VisionTargeting extends Command {
  private double m_currentAngle;
  private double m_currentRpms;

  public VisionTargeting() {
    addRequirements(Robot.pivot, Robot.shooter);
  }

  @Override
  public void execute() {
    double yaw = Robot.shooterCam.getSpeakerTargetYaw();

    if (Double.isNaN(yaw)) {
      return;
    }

    double pitch = Robot.shooterCam.getSpeakerTargetPitch();
    updateVisionTargeting(pitch);
    Robot.pivot.setAngle(m_currentAngle);
    Robot.shooter.setRPM(m_currentRpms);

    SmartDashboard.putNumber("camera pitch", pitch);
    SmartDashboard.putNumber("Pivot vision setpoint", m_currentAngle);
    SmartDashboard.putNumber("Shooter vision RPMs", m_currentRpms);
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

  private void updateVisionTargeting(double pitch) {
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, pitch);
    if (curveIndex == -1) {
      // System.err.println("[VisionTargeting] pitch out of range");
      m_currentAngle = Double.NaN;
      m_currentRpms = Double.NaN;
      return;
    }

    double[] high = Robot.shooterCurve[curveIndex];
    double[] low = Robot.shooterCurve[curveIndex + 1];

    double highPitch = high[0];
    double lowPitch = low[0];
    double highPivotRotation = high[1];
    double lowPivotRotation = low[1];
    double highShooterRPM = high[2];
    double lowShooterRPM = low[2];

    m_currentAngle =
        RobotMath.linearlyInterpolate(
            highPivotRotation, lowPivotRotation, highPitch, lowPitch, pitch);
    m_currentRpms =
        RobotMath.linearlyInterpolate(highShooterRPM, lowShooterRPM, highPitch, lowPitch, pitch);
  }
}
