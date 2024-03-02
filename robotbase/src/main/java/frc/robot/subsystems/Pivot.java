package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIVOT;
import frc.robot.util.Utility;

public class Pivot extends SubsystemBase {
  private CANSparkMax m_pivotMotor;
  private SparkPIDController m_pivotPIDController;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private double m_targetAngle;
  private double m_zeroOffset;

  public Pivot() {
    m_pivotMotor = new CANSparkMax(Constants.CAN_ID.PIVOT_MOTOR_ID, MotorType.kBrushless);
    m_targetAngle = Double.NaN;
    configure();

    m_zeroOffset = Preferences.getDouble(Constants.PIVOT.PREFERENCES_ZERO_OFFSET_KEY, Double.NaN);
    if (!Double.isNaN(m_zeroOffset)) {
      setZeroOffset(m_zeroOffset);
    }
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
    return !Double.isNaN(m_zeroOffset);
  }

  public boolean isSettingAngle() {
    return !Double.isNaN(m_targetAngle);
  }

  public void setAngle(double angle) {
    if (!isZeroed()) {
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
    m_pivotPIDController.setReference(m_targetAngle, ControlType.kPosition);
  }

  public void setSpeed(double speed) {
    m_pivotMotor.set(speed);
    m_targetAngle = Double.NaN;
  }

  public void stop() {
    m_pivotMotor.stopMotor();
    m_targetAngle = Double.NaN;
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
    setZeroOffset(newOffset);
    System.out.println("[Pivot] Zero Set");
  }

  private void setZeroOffset(double newOffset) {
    m_absoluteEncoder.setZeroOffset(newOffset);
    m_zeroOffset = newOffset;
    Preferences.setDouble(Constants.PIVOT.PREFERENCES_ZERO_OFFSET_KEY, newOffset);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot angle", getCurrentAngle());
    SmartDashboard.putNumber("Pivot target angle", getTargetAngle());
  }
}
