package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CLIMBER;

public class Climber extends SubsystemBase {
  private CANSparkMax m_rightClimberMotor;
  private CANSparkMax m_leftClimberMotor;

  public Climber() {
    m_rightClimberMotor =
        new CANSparkMax(Constants.CAN_ID.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    m_leftClimberMotor =
        new CANSparkMax(Constants.CAN_ID.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);

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

  public void set(double right, double left) {
    m_rightClimberMotor.set(right);
    m_leftClimberMotor.set(left);
  }

  public void setAxis(double rightAxisSpeed, double leftAxisSpeed) {
    double right = rightAxisSpeed * CLIMBER.AXIS_MAX_SPEED;
    double left = leftAxisSpeed * CLIMBER.AXIS_MAX_SPEED;
    set(right, left);
  }

  public void stop() {
    set(0, 0);
  }

  public double getRightAmps() {
    return m_rightClimberMotor.getOutputCurrent();
  }

  public double getLeftAmps() {
    return m_leftClimberMotor.getOutputCurrent();
  }

  public boolean isRightAtCurrent(double amps) {
    return getRightAmps() >= amps;
  }

  public boolean isLeftAtCurrent(double amps) {
    return getLeftAmps() >= amps;
  }

  private double getRotations(CANSparkMax motor) {
    return motor.getEncoder().getPosition();
  }

  public boolean isPastRotations(double rotations, int direction) {
    switch (direction) {
      case 1:
        return getRotations(m_rightClimberMotor) >= rotations
            && getRotations(m_leftClimberMotor) >= rotations;
      case -1:
        return getRotations(m_rightClimberMotor) <= rotations
            && getRotations(m_leftClimberMotor) <= rotations;
      default:
        return true;
    }
  }
}
