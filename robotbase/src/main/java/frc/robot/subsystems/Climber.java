package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.APRIL_TAG_IDS;
import frc.robot.Constants.SHOOTER_PHOTON_CAMERA;
import frc.robot.Robot;

public class Climber extends SubsystemBase {
  private CANSparkMax m_rightClimberMotor;
  private CANSparkMax m_leftClimberMotor;

  public Climber() {
    m_rightClimberMotor = new CANSparkMax(Constants.CAN_ID.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    m_leftClimberMotor = new CANSparkMax(Constants.CAN_ID.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);

    configure();
  }

  public void configure() {
    m_rightClimberMotor.setInverted(Constants.CLIMBER.RIGHT_MOTOR_INVERTED);
    m_rightClimberMotor.setIdleMode(IdleMode.kBrake);
    m_rightClimberMotor.setSmartCurrentLimit(
        Constants.CLIMBER.MOTOR_STALL_LIMIT_AMPS, Constants.CLIMBER.MOTOR_FREE_LIMIT_AMPS);
    m_rightClimberMotor.enableVoltageCompensation(12);

    m_leftClimberMotor.setInverted(Constants.CLIMBER.LEFT_MOTOR_INVERTED);
    m_leftClimberMotor.setIdleMode(IdleMode.kBrake);
    m_leftClimberMotor.setSmartCurrentLimit(
        Constants.CLIMBER.MOTOR_STALL_LIMIT_AMPS, Constants.CLIMBER.MOTOR_FREE_LIMIT_AMPS);
    m_leftClimberMotor.enableVoltageCompensation(12);
  }

  public void setSpeed(double leftSpeed, double rightSpeed) {
    m_leftClimberMotor.set(leftSpeed);
    m_rightClimberMotor.set(rightSpeed);
  }

  public void setAmpLimits(int freeLimit, int stallLimit) {
    m_leftClimberMotor.setSmartCurrentLimit(stallLimit, freeLimit);
    m_rightClimberMotor.setSmartCurrentLimit(stallLimit, freeLimit);
  }

  public void stop() {
    setSpeed(0, 0);
  }

  public double getRightVelocity() {
    return m_rightClimberMotor.getEncoder().getVelocity();
  }

  public double getLeftVelocity() {
    return m_leftClimberMotor.getEncoder().getVelocity();
  }

  public double getRightRotations() {
    return m_rightClimberMotor.getEncoder().getPosition();
  }

  public double getLeftRotations() {
    return m_leftClimberMotor.getEncoder().getPosition();
  }

  public boolean isPastRotations(double rotations, int direction) {
    switch (direction) {
      case 1:
        return getLeftRotations() >= rotations && getRightRotations() >= rotations;
      case -1:
        return getLeftRotations() <= rotations && getRightRotations() <= rotations;
      default:
        return true;
    }
  }

  public void setZero() {
    m_rightClimberMotor.getEncoder().setPosition(0.0);
    m_leftClimberMotor.getEncoder().setPosition(0.0);
  }

  public double getStageLineupRotationSetpoint() {
    if (Robot.shooterCam.getPipeline() != SHOOTER_PHOTON_CAMERA.APRIL_TAG_PIPELINE) {
      return Double.NaN;
    }

    ArrayList<PhotonTrackedTarget> stageTargets = Robot.shooterCam.filterAprilTags(APRIL_TAG_IDS.STAGE_TAGS);
    if (stageTargets == null || stageTargets.size() == 0) {
      return Double.NaN;
    }

    PhotonTrackedTarget closestTarget = stageTargets.get(0);
    return SHOOTER_PHOTON_CAMERA.STAGE_APRILTAG_ROTATION_SETPOINTS.get(
        closestTarget.getFiducialId());
  }
}
