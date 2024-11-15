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
  private boolean m_gotIt;

  public DrivePickup() {
    m_startingPipeline = Robot.intakeCam.getPipeline();
    m_yawController = SWERVE.VISION_X_TRANSLATION_PID_CONTROLLER;
    addRequirements(Robot.swerve, Robot.intakeCam);
  }

  @Override
  public void initialize() {
    m_yawController.setTolerance(SWERVE.VISION_YAW_TOLERANCE);
    m_yawController.setSetpoint(SWERVE.TRANSLATE_TO_GAMEPIECE_YAW_SETPOINT);
    m_yawController.reset();
    m_lastPitch = Double.NaN;
    m_gotIt = false;
  }

  @Override
  public void execute() {
    double targetYaw = Robot.intakeCam.getNoteTargetYaw();
    double targetPitch = Robot.intakeCam.getNoteTargetPitch();

    double stickX = Robot.driverControls.getX() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
    double stickY = Robot.driverControls.getY() * CompSwerveTunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
    double stickRotation =
        Robot.driverControls.getRotation() * Constants.SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND;

    if (Robot.driverControls.getLeftTrigger() >= CONTROLLER.DRIVE_TRANSLATE_INTAKE_THRESHOLD) {

      executeAutoPickup(targetPitch, targetYaw);
    } else if (Robot.driverControls.isLeftTriggerPressed()) {
      executeTargetLock(targetPitch, targetYaw, stickX, stickY);
    } else {
      executeDriveNormal(stickX, stickY, stickRotation);
    }

    if (!Double.isNaN(targetPitch)) {
      m_lastPitch = targetPitch;
    }
  }

  private void executeCreepForward() {
    Robot.swerve.driveRobotRelative(SWERVE.TELEOP_CREEP_TO_GAMEPIECE_Y_METERS_PER_SECOND, 0, 0);
  }

  private void executeAutoPickup(double targetPitch, double targetYaw) {
    if (Double.isNaN(targetYaw)) {
      executeCreepForward();
      return;
    }

    boolean pitchInRange = targetPitch >= -16.0 && targetPitch <= 5.0;
    boolean yawInRange = Math.abs(targetYaw) <= Constants.SWERVE.AUTO_INTAKE_YAW_TOLERANCE;
    if (targetPitch < m_lastPitch && m_lastPitch < -16.0) {
      // m_gotIt = true;
    }

    if (m_gotIt) {
      executeCreepForward();
      return;
    }

    if (!pitchInRange || !yawInRange) {
      Robot.swerve.stopMotors();
      return;
    }

    if (Utility.isWithinTolerance(
        targetYaw, m_yawController.getSetpoint(), SWERVE.TRANSLATE_TO_GAMEPIECE_YAW_TOLERANCE)) {
      targetYaw = m_yawController.getSetpoint();
    }

    double xMetersPerSecond = m_yawController.calculate(targetYaw);

    Robot.swerve.driveRobotRelative(
        SWERVE.TELEOP_DRIVE_TO_GAMEPIECE_Y_METERS_PER_SECOND, xMetersPerSecond, 0);
  }

  private void executeTargetLock(
      double targetPitch, double targetYaw, double stickX, double stickY) {
    boolean pitchInRange = targetPitch >= -16.0 && targetPitch <= 5.0;
    boolean yawInRange = targetYaw >= -26.0 && targetYaw <= 26.0;

    if (pitchInRange) {
      Robot.swerve.driveFieldRelative(stickY, stickX, 0);
    }

    Robot.swerve.driveTargetLock(stickY, stickX, yawInRange ? targetYaw : 0, 0, yawInRange);
  }

  private void executeDriveNormal(double stickX, double stickY, double stickRotation) {
    if (stickX == 0 && stickY == 0 && stickRotation == 0) {
      Robot.swerve.stopMotors();
    } else {
      Robot.swerve.driveFieldRelative(stickY, stickX, stickRotation);
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
    Robot.intakeCam.setPipeline(m_startingPipeline); 
  }
}
