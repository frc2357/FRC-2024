package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TRAP_AMP_ARM;

public class TrapAmpArmSubsystem extends SubsystemBase {

  private CANSparkMax m_motor;

  public TrapAmpArmSubsystem() {
    m_motor = new CANSparkMax(Constants.CAN_ID.TRAP_AMP_ARM_MOTOR_ID, MotorType.kBrushless);
    configure();
  }

  private void configure() {
    m_motor.setInverted(TRAP_AMP_ARM.MOTOR_IS_INVERTED);
    m_motor.setIdleMode(TRAP_AMP_ARM.MOTOR_IDLE_MODE);
    m_motor.setSmartCurrentLimit(
        TRAP_AMP_ARM.MOTOR_STALL_LIMIT_AMPS, TRAP_AMP_ARM.MOTOR_FREE_LIMIT_AMPS);
    m_motor.enableVoltageCompensation(12);
  }

  public void set(double speed) {
    m_motor.set(speed);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public void setAxisSpeed(double axisSpeed) {
    double motorSpeed = (-axisSpeed) * TRAP_AMP_ARM.AXIS_MAX_SPEED;
    m_motor.set(motorSpeed);
  }

  public double getPosition() {
    return m_motor.getEncoder().getPosition();
  }
}
