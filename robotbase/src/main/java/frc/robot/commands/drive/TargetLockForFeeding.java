package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CompSwerveTunerConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class TargetLockForFeeding extends Command {
  public int m_startingPipeline;
  private double m_yawSetpoint;

  public TargetLockForFeeding() {
    m_startingPipeline = Robot.shooterCam.getPipeline();
    addRequirements(Robot.swerve, Robot.shooterCam);
  }

  @Override
  public void initialize() {
    Robot.shooterCam.setAprilTagPipelineActive();
    if (Robot.state.getAlliance() == Alliance.Blue) {
      m_yawSetpoint = Constants.SWERVE.BLUE_PASSING_WALL_SIDE_YAW_SETPOINT;
    } else if (Robot.state.getAlliance() == Alliance.Red) {
      m_yawSetpoint = Constants.SWERVE.RED_PASSING_WALL_SIDE_YAW_SETPOINT;
    }
  }

  @Override
  public void execute() {
    double pitch = Robot.shooterCam.getCenterStageTargetPitch();
    if (Double.isNaN(pitch)) {
      Robot.swerve.driveTargetLock(
          Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
          Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
          0,
          0,
          false);
      return;
    }

    double yaw = Robot.shooterCam.getCenterStageTargetYaw();
    double robotYaw = Math.abs(Robot.swerve.getYaw() % 360);
    if (Robot.state.getAlliance() == Alliance.Blue) {
      if (robotYaw < Constants.SWERVE.PASSING_MID_FIELD_ROBOT_YAW_TOLERANCE) {
        m_yawSetpoint = Constants.SWERVE.BLUE_PASSING_MID_FIELD_YAW_SETPOINT;
      } else if (robotYaw > Constants.SWERVE.PASSING_WALL_SIDE_ROBOT_YAW_TOLERANCE) {
        m_yawSetpoint = Constants.SWERVE.BLUE_PASSING_WALL_SIDE_YAW_SETPOINT;
      }
    } else if (Robot.state.getAlliance() == Alliance.Red) {
      if (robotYaw < Constants.SWERVE.PASSING_MID_FIELD_ROBOT_YAW_TOLERANCE) {
        m_yawSetpoint = Constants.SWERVE.RED_PASSING_MID_FIELD_YAW_SETPOINT;
      } else if (robotYaw > Constants.SWERVE.PASSING_WALL_SIDE_ROBOT_YAW_TOLERANCE) {
        m_yawSetpoint = Constants.SWERVE.RED_PASSING_WALL_SIDE_YAW_SETPOINT;
      }
    }
    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
        Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
        !Double.isNaN(yaw) ? yaw : 0,
        m_yawSetpoint,
        !Double.isNaN(yaw));
  } 
}
