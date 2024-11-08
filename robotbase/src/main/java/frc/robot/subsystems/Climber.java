package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CLIMBER;

public class Climber extends SubsystemBase {
  private SparkMax m_rightClimberMotor;
  private SparkMax m_leftClimberMotor;

  public Climber() {
    m_rightClimberMotor =
        new SparkMax(Constants.CAN_ID.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    m_leftClimberMotor =
        new SparkMax(Constants.CAN_ID.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);

    configure();
  }

  public void configure() {
    var leftMotorConfig = new SparkMaxConfig().inverted(CLIMBER.LEFT_MOTOR_INVERTED).idleMode(IdleMode.kBrake).smartCurrentLimit(CLIMBER.MOTOR_STALL_LIMIT_AMPS, CLIMBER.MOTOR_FREE_LIMIT_AMPS).voltageCompensation(12);
    var rightMotorConfig = new SparkMaxConfig().inverted(CLIMBER.RIGHT_MOTOR_INVERTED).idleMode(IdleMode.kBrake).smartCurrentLimit(CLIMBER.MOTOR_STALL_LIMIT_AMPS, CLIMBER.MOTOR_FREE_LIMIT_AMPS).voltageCompensation(12);
    m_leftClimberMotor.configure(leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightClimberMotor.configure(rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setSpeed(double leftSpeed, double rightSpeed) {
    m_leftClimberMotor.set(leftSpeed);
    m_rightClimberMotor.set(rightSpeed);
  }

  public void setAmpLimits(int freeLimit, int stallLimit) {
    var leftMotorConfig = new SparkMaxConfig().inverted(CLIMBER.LEFT_MOTOR_INVERTED).idleMode(IdleMode.kBrake).smartCurrentLimit(stallLimit, freeLimit).voltageCompensation(12);
    var rightMotorConfig = new SparkMaxConfig().inverted(CLIMBER.RIGHT_MOTOR_INVERTED).idleMode(IdleMode.kBrake).smartCurrentLimit(stallLimit, freeLimit).voltageCompensation(12);
    m_leftClimberMotor.configure(leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightClimberMotor.configure(rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

  public void setZero() {
    m_rightClimberMotor.getEncoder().setPosition(0.0);
    m_leftClimberMotor.getEncoder().setPosition(0.0);
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
}
