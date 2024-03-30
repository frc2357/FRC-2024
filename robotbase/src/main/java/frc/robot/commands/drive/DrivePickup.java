package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CompSwerveTunerConstants;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.util.Utility;

public class DrivePickup extends Command {
  private int m_startingPipeline;
  private PIDController m_yawController;
  private double m_lastPitch;

  public DrivePickup() {
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

    m_lastPitch = Double.NaN;
  }

  @Override
  public void execute() {
    double targetYaw = Robot.intakeCam.getNoteTargetYaw();
    double pitch = Robot.intakeCam.getNoteTargetPitch();

    double stickX = Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12VoltsMps;
    double stickY = Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12VoltsMps;
    double stickRotation = Robot.driverControls.getRotation() * Constants.SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND;

    if (Robot.driverControls.getLeftTrigger() >= CONTROLLER.DRIVE_TRANSLATE_INTAKE_THRESHOLD) {

      if (Double.isNaN(targetYaw) ||
          (pitch < -16 || pitch > 5) ||
          (!Double.isNaN(m_lastPitch) && m_lastPitch <= -16)) {
        System.out.println("[TranslateToGamepiece] No gamepiece detected");
        // Continue driving forward if we don't see a gamepiece
        Robot.swerve.driveRobotRelative(
            SWERVE.TELEOP_TRANSLATE_TO_GAMEPIECE_Y_METERS_PER_SECOND, 0, 0);
        return;
      }

      if (Utility.isWithinTolerance(
          targetYaw, m_yawController.getSetpoint(), SWERVE.TRANSLATE_TO_GAMEPIECE_YAW_TOLERANCE)) {
        targetYaw = m_yawController.getSetpoint();
      }

      double xMetersPerSecond = m_yawController.calculate(targetYaw);

      Robot.swerve.driveRobotRelative(
          SWERVE.TELEOP_TRANSLATE_TO_GAMEPIECE_Y_METERS_PER_SECOND, xMetersPerSecond, 0);

      m_lastPitch = pitch;
      return;
    }

    if (Robot.driverControls.isLeftTriggerPressed()) {
      if (!Double.isNaN(m_lastPitch) && m_lastPitch <= -16) {
        Robot.swerve.driveFieldRelative(stickY,
            stickX, 0);
      }

      Robot.swerve.driveTargetLock(
          stickY,
          stickX,
          !Double.isNaN(targetYaw) ? targetYaw : 0,
          0,
          !Double.isNaN(targetYaw));

      m_lastPitch = pitch;
      return;
    }

    if (stickX == 0 && stickY == 0 && stickRotation == 0) {
      Robot.swerve.stopMotors();
    } else {
      Robot.swerve.driveFieldRelative(
          stickY,
          stickX,
          stickRotation);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
    Robot.intakeCam.setPipeline(m_startingPipeline);
  }
}
