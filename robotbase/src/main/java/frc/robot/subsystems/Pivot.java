package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIVOT;
import frc.robot.util.Utility;

public class Pivot extends SubsystemBase {
  private CANSparkMax m_pivotMotor;
  private SparkPIDController m_pivotPIDController;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private double m_targetAngle;
  private double m_axisSpeed;
  private boolean m_isZeroed;

  public Pivot() {
    m_pivotMotor = new CANSparkMax(Constants.CAN_ID.PIVOT_MOTOR_ID, MotorType.kBrushless);
    m_targetAngle = Double.NaN;
    m_axisSpeed = Double.NaN;
    m_isZeroed = false;
    configure();
  }

  private void configure() {
    m_pivotMotor.setInverted(PIVOT.MOTOR_INVERTED);
    m_pivotMotor.setIdleMode(PIVOT.IDLE_MODE);
    m_pivotMotor.setSmartCurrentLimit(PIVOT.MOTOR_STALL_LIMIT_AMPS, PIVOT.MOTOR_FREE_LIMIT_AMPS);
    m_pivotMotor.enableVoltageCompensation(12);
    m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    m_pivotPIDController = m_pivotMotor.getPIDController();

    m_pivotPIDController.setP(PIVOT.PIVOT_P);
    m_pivotPIDController.setI(PIVOT.PIVOT_I);
    m_pivotPIDController.setD(PIVOT.PIVOT_D);
    m_pivotPIDController.setFF(PIVOT.PIVOT_FF);

    m_pivotPIDController.setOutputRange(-1, 1);

    m_absoluteEncoder = m_pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_absoluteEncoder.setInverted(PIVOT.ENCODER_INVERTED);
    m_absoluteEncoder.setPositionConversionFactor(PIVOT.ENCODER_POSITION_CONVERSION_FACTOR);
    m_absoluteEncoder.setVelocityConversionFactor(PIVOT.ENCODER_VELOCITY_CONVERSION_FACTOR);
    m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    m_pivotPIDController.setFeedbackDevice(m_absoluteEncoder);
  }

  public boolean isZeroed() {
    return m_isZeroed;
  }

  public boolean isStopped() {
    return Double.isNaN(m_targetAngle) && Double.isNaN(m_axisSpeed);
  }

  public boolean isSettingAngle() {
    return !Double.isNaN(m_targetAngle);
  }

  public boolean isRunningAxisSpeed() {
    return !Double.isNaN(m_axisSpeed);
  }

  public void setAngle(double angle) {
    if (!m_isZeroed) {
      System.err.println("PIVOT: Cannot set angle, Pivot not zeroed!");
      return;
    }
    if (angle < Constants.PIVOT.MIN_PIVOT_ANGLE) {
      System.err.println("PIVOT: Cannot set angle lower than minimum");
      return;
    }
    if (angle > Constants.PIVOT.MAX_PIVOT_ANGLE) {
      System.err.println("PIVOT: Cannot set angle higher than minimum");
      return;
    }
    m_targetAngle = angle;
    m_axisSpeed = Double.NaN;
    m_pivotPIDController.setReference(m_targetAngle, ControlType.kPosition);
  }

  public void setAxisSpeed(double axisSpeed) {
    m_axisSpeed = axisSpeed;
    m_targetAngle = Double.NaN;
    double motorSpeed = (-axisSpeed) * PIVOT.AXIS_MAX_SPEED;
    m_pivotMotor.set(motorSpeed);
  }

  public void stop() {
    m_pivotMotor.stopMotor();
    m_targetAngle = Double.NaN;
    m_axisSpeed = Double.NaN;
  }

  public double getVelocity() {
    return m_absoluteEncoder.getVelocity();
  }

  public double getTargetAngle() {
    return m_targetAngle;
  }

  public double getCurrentAngle() {
    return m_absoluteEncoder.getPosition();
  }

  public boolean isPivotAtAngle() {
    return Utility.isWithinTolerance(
        getCurrentAngle(), m_targetAngle, PIVOT.POSITION_ALLOWED_ERROR);
  }

  public void setZero() {
    double currentOffset = m_absoluteEncoder.getZeroOffset();
    double newOffset = getCurrentAngle() + currentOffset - Constants.PIVOT.MIN_PIVOT_ANGLE;
    newOffset %= 360;
    m_absoluteEncoder.setZeroOffset(newOffset);
    m_isZeroed = true;
  }
}
