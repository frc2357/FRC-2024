package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TRAP_AMP_ARM;
import frc.robot.util.Utility;

public class TrapAmpArm extends SubsystemBase {
  private boolean m_isClosedLoopEnabled = false;

  private CANSparkMax m_motor;
  private SparkPIDController m_PIDController;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private double m_targetRotations;

  public TrapAmpArm() {
    m_motor = new CANSparkMax(Constants.CAN_ID.TRAP_AMP_ARM_MOTOR_ID, MotorType.kBrushless);
    configure();
  }

  private void configure() {
    m_motor.setInverted(TRAP_AMP_ARM.MOTOR_IS_INVERTED);
    m_motor.setIdleMode(TRAP_AMP_ARM.MOTOR_IDLE_MODE);
    m_motor.setSmartCurrentLimit(
        TRAP_AMP_ARM.MOTOR_STALL_LIMIT_AMPS, TRAP_AMP_ARM.MOTOR_FREE_LIMIT_AMPS);
    m_motor.enableVoltageCompensation(12);

    m_PIDController = m_motor.getPIDController();

    m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_PIDController.setP(TRAP_AMP_ARM.MOTOR_PID_P);
    m_PIDController.setI(TRAP_AMP_ARM.MOTOR_PID_I);
    m_PIDController.setD(TRAP_AMP_ARM.MOTOR_PID_D);
    m_PIDController.setFF(TRAP_AMP_ARM.MOTOR_PID_FF);

    m_PIDController.setOutputRange(-1, 1);
    m_PIDController.setSmartMotionMaxVelocity(TRAP_AMP_ARM.SMART_MOTION_MAX_VEL_RPM, 0);
    m_PIDController.setSmartMotionMinOutputVelocity(TRAP_AMP_ARM.SMART_MOTION_MIN_VEL_RPM, 0);
    m_PIDController.setSmartMotionMaxAccel(TRAP_AMP_ARM.SMART_MOTION_MAX_ACC_RPM, 0);
    m_PIDController.setSmartMotionAllowedClosedLoopError(
        TRAP_AMP_ARM.SMART_MOTION_ALLOWED_ERROR, 0);

    m_absoluteEncoder.setInverted(TRAP_AMP_ARM.ABSOLUTE_ENCODER_IS_INVERTED);
    m_absoluteEncoder.setPositionConversionFactor(
        TRAP_AMP_ARM.ABSOLUTE_ENCODER_POSITION_SCALE_FACTOR);
    m_absoluteEncoder.setVelocityConversionFactor(
        TRAP_AMP_ARM.ABSOLUTE_ENCODER_VELOCITY_SCALE_FACTOR);
  }

  public void set(double speed) {
    m_isClosedLoopEnabled = false;
    m_motor.set(speed);
  }

  public void setTargetRotations(double targetRotations) {
    m_targetRotations = targetRotations;
    m_isClosedLoopEnabled = true;
    m_PIDController.setReference(m_targetRotations, ControlType.kSmartMotion);
  }

  public void stop() {
    m_isClosedLoopEnabled = false;
    m_motor.stopMotor();
  }

  public void setAxisSpeed(double axisSpeed) {
    m_isClosedLoopEnabled = false;
    double motorSpeed = (-axisSpeed) * TRAP_AMP_ARM.AXIS_MAX_SPEED;
    m_motor.set(motorSpeed);
  }

  public double getMotorRotations() {
    return m_absoluteEncoder.getPosition();
  }

  public double getMotorVelocity() {
    return m_absoluteEncoder.getVelocity();
  }

  public boolean isAtTargetRotations() {
    return Utility.isWithinTolerance(
        getMotorRotations(), m_targetRotations, TRAP_AMP_ARM.SMART_MOTION_ALLOWED_ERROR);
  }

  @Override
  public void periodic() {
    if (m_isClosedLoopEnabled && isAtTargetRotations()) {
      m_isClosedLoopEnabled = false;
    }
  }
}
