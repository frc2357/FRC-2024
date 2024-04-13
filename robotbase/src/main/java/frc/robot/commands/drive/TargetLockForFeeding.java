package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CompSwerveTunerConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class TargetLockForFeeding extends Command {
  public int m_startingPipeline;

  public TargetLockForFeeding() {
    m_startingPipeline = Robot.shooterCam.getPipeline();
    addRequirements(Robot.swerve, Robot.shooterCam);
  }

  @Override
  public void initialize() {
    Robot.shooterCam.setAprilTagPipelineActive();
  }

  @Override
  public void execute() {
    double pitch = Robot.shooterCam.getCenterStageTargetPitch();
    if (Double.isNaN(pitch)) {
      Robot.swerve.driveTargetLock(
          Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
          Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
          0,
          0,
          false);
      return;
    }

    double yaw = Robot.shooterCam.getCenterStageTargetYaw();
    double yawSetpoint = Constants.SWERVE.FEEDING_TARGET_LOCK_YAW_SETPOINT;
    if (Robot.state.getAlliance() == Alliance.Blue) {
      yawSetpoint *= -1;
    }
    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
        Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
        !Double.isNaN(yaw) ? yaw : 0,
        yawSetpoint,
        !Double.isNaN(yaw));
  }
}
