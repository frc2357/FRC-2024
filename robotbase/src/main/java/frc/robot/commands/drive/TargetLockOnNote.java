package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CompSwerveTunerConstants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.util.Utility;

public class TargetLockOnNote extends Command {
  private int m_startingPipeline;
  private PIDController m_yawController;

  public TargetLockOnNote() {
    m_startingPipeline = Robot.intakeCam.getPipeline();
    m_yawController = SWERVE.VISION_X_TRANSLATION_PID_CONTROLLER;
    addRequirements(Robot.swerve, Robot.intakeCam);
  }

  @Override
  public void initialize() {
    Robot.intakeCam.setNeuralNetworkPipelineActive();

    m_yawController.setTolerance(SWERVE.VISION_YAW_TOLERANCE);
    m_yawController.setSetpoint(SWERVE.TRANSLATE_TO_GAMEPIECE_YAW_SETPOINT);
    m_yawController.reset();
  }

  @Override
  public void execute() {
    double targetYaw = Robot.intakeCam.getNoteTargetYaw();

    if (Robot.driverControls.getLeftTrigger() < CONTROLLER.DRIVE_TRANSLATE_INTAKE_THRESHOLD) {
      Robot.swerve.driveTargetLock(
          Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
          Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12VoltsMps,
          !Double.isNaN(targetYaw) ? targetYaw : 0,
          0,
          !Double.isNaN(targetYaw));
    } else {
      double pitch = Robot.intakeCam.getNoteTargetPitch();

      if (Double.isNaN(targetYaw) || (pitch < -16 || pitch > 5)) {
        System.out.println("[TranslateToGamepiece] No gamepiece detected");
        // Continue driving forward and rotating even if we don't see a gamepiece
        Robot.swerve.driveRobotRelative(0, 0, 0);
      }

      if (Utility.isWithinTolerance(
          targetYaw, m_yawController.getSetpoint(), SWERVE.TRANSLATE_TO_GAMEPIECE_YAW_TOLERANCE)) {
        targetYaw = m_yawController.getSetpoint();
      }

      double xMetersPerSecond = m_yawController.calculate(targetYaw);

      Robot.swerve.driveRobotRelative(
          SWERVE.TELEOP_TRANSLATE_TO_GAMEPIECE_Y_METERS_PER_SECOND, xMetersPerSecond, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
    Robot.intakeCam.setPipeline(m_startingPipeline);
  }
}
