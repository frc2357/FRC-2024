package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.util.Utility;

public class ExtensionArm extends SubsystemBase {
  private boolean m_isClosedLoopEnabled = false;

  private CANSparkMax m_motor;
  private SparkPIDController m_PIDController;
  private RelativeEncoder m_encoder;
  private double m_targetRotations;

  public ExtensionArm() {
    m_motor = new CANSparkMax(Constants.CAN_ID.TRAP_AMP_ARM_MOTOR_ID, MotorType.kBrushless);
    configure();
  }

  private void configure() {
    m_motor.setInverted(EXTENSION_ARM.MOTOR_IS_INVERTED);
    m_motor.setIdleMode(EXTENSION_ARM.MOTOR_IDLE_MODE);
    m_motor.setSmartCurrentLimit(
        EXTENSION_ARM.MOTOR_STALL_LIMIT_AMPS, EXTENSION_ARM.MOTOR_FREE_LIMIT_AMPS);
    m_motor.enableVoltageCompensation(12);

    m_encoder = m_motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    m_encoder.setInverted(Constants.EXTENSION_ARM.ENCODER_INVERTED);

    m_PIDController = m_motor.getPIDController();

    m_PIDController.setP(EXTENSION_ARM.MOTOR_PID_P);
    m_PIDController.setI(EXTENSION_ARM.MOTOR_PID_I);
    m_PIDController.setD(EXTENSION_ARM.MOTOR_PID_D);
    m_PIDController.setFF(EXTENSION_ARM.MOTOR_PID_FF);

    m_PIDController.setFeedbackDevice(m_encoder);

    m_PIDController.setOutputRange(-1, 1);
    m_PIDController.setSmartMotionMaxVelocity(EXTENSION_ARM.SMART_MOTION_MAX_VEL_RPM, 0);
    m_PIDController.setSmartMotionMinOutputVelocity(EXTENSION_ARM.SMART_MOTION_MIN_VEL_RPM, 0);
    m_PIDController.setSmartMotionMaxAccel(EXTENSION_ARM.SMART_MOTION_MAX_ACC_RPM, 0);
    m_PIDController.setSmartMotionAllowedClosedLoopError(
        EXTENSION_ARM.SMART_MOTION_ALLOWED_ERROR, 0);
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

  public void zeroArm() {
    m_encoder.setPosition(0);
  }

  public void setAxisSpeed(double axisSpeed) {
    m_isClosedLoopEnabled = false;
    double motorSpeed = (-axisSpeed) * EXTENSION_ARM.AXIS_MAX_SPEED;
    m_motor.set(motorSpeed);
  }

  public double getRotations() {
    return m_encoder.getPosition();
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public boolean isAtTargetRotations() {
    return Utility.isWithinTolerance(
        getRotations(), m_targetRotations, EXTENSION_ARM.SMART_MOTION_ALLOWED_ERROR);
  }

  @Override
  public void periodic() {
    if (m_isClosedLoopEnabled && isAtTargetRotations()) {
      m_isClosedLoopEnabled = false;
    }

    SmartDashboard.putNumber("Arm position", getRotations());
  }
}
