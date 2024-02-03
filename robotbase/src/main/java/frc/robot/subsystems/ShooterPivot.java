package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SHOOTER_PIVOT;
import frc.robot.util.Utility;

public class ShooterPivot extends SubsystemBase {
  private boolean m_isClosedLoopEnabled = false;

  private CANSparkMax m_pivotMotor;
  private SparkPIDController m_pivotPIDController;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private double m_targetRotations;

  public ShooterPivot() {
    m_pivotMotor = new CANSparkMax(Constants.CAN_ID.SHOOTER_PIVOT_MOTOR_ID, MotorType.kBrushless);
    configure();
  }

  private void configure() {
    m_pivotMotor.setInverted(SHOOTER_PIVOT.MOTOR_INVERTED);
    m_pivotMotor.setIdleMode(SHOOTER_PIVOT.IDLE_MODE);
    m_pivotMotor.setSmartCurrentLimit(
        SHOOTER_PIVOT.MOTOR_STALL_LIMIT_AMPS,
        SHOOTER_PIVOT.MOTOR_FREE_LIMIT_AMPS);
    m_pivotMotor.enableVoltageCompensation(12);

    m_pivotPIDController = m_pivotMotor.getPIDController();

    m_pivotPIDController.setP(SHOOTER_PIVOT.PIVOT_P);
    m_pivotPIDController.setI(SHOOTER_PIVOT.PIVOT_I);
    m_pivotPIDController.setD(SHOOTER_PIVOT.PIVOT_D);
    m_pivotPIDController.setFF(SHOOTER_PIVOT.PIVOT_FF);

    m_pivotPIDController.setOutputRange(-1, 1);
    m_absoluteEncoder = m_pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_absoluteEncoder.setInverted(SHOOTER_PIVOT.ENCODER_INVERTED);
    m_absoluteEncoder.setPositionConversionFactor(
        SHOOTER_PIVOT.ENCODER_POSITION_CONVERSION_FACTOR);
    m_absoluteEncoder.setVelocityConversionFactor(
        SHOOTER_PIVOT.ENCODER_VELOCITY_CONVERSION_FACTOR);
  }

  public void setPivotRotations(double rotations) {
    m_isClosedLoopEnabled = true;
    m_targetRotations = rotations;
    m_pivotPIDController.setReference(m_targetRotations, ControlType.kPosition);
    m_pivotPIDController.setPositionPIDWrappingEnabled(SHOOTER_PIVOT.POSITION_PID_WRAPPING_ENABLED);
  }

  public void setPivotAxisSpeed(double axisSpeed) {
    m_isClosedLoopEnabled = false;
    double motorSpeed = (-axisSpeed) * SHOOTER_PIVOT.AXIS_MAX_SPEED;
    m_pivotMotor.set(motorSpeed);
  }

  public void stop() {
    m_isClosedLoopEnabled = false;
    m_pivotMotor.stopMotor();
  }

  public void resetEncoder() {
    m_pivotMotor.getEncoder().setPosition(0);
    m_targetRotations = 0;
  }

  public double getPivotRotations() {
    return m_pivotMotor.getEncoder().getPosition();
  }

  public double getAngle() {
    return m_absoluteEncoder.getPosition();
  }

  public boolean isPivotAtRotations() {
    return Utility.isWithinTolerance(
        getAngle(), m_targetRotations, SHOOTER_PIVOT.POSITION_ALLOWED_ERROR);
  }

  @Override
  public void periodic() {
    if (m_isClosedLoopEnabled && isPivotAtRotations()) {
      m_isClosedLoopEnabled = false;
    }
  }
}
