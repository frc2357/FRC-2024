package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIVOT;
import frc.robot.util.Utility;

public class Pivot extends SubsystemBase {
  private SparkMax m_pivotMotor;
  private SparkClosedLoopController m_pivotPIDController;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private double m_targetAngle;

  public Pivot() {
    m_pivotMotor = new SparkMax(Constants.CAN_ID.PIVOT_MOTOR_ID, MotorType.kBrushless);
    m_targetAngle = Double.NaN;
    configure();

    // m_zeroOffset = Preferences.getDouble(Constants.PIVOT.PREFERENCES_ZERO_OFFSET_KEY,
    // Double.NaN);
    // if (!Double.isNaN(m_zeroOffset)) {
    //   setZeroOffset(m_zeroOffset);
    // }

    SmartDashboard.putNumber(PIVOT.PIVOT_OFFSET_KEY, 0.0);
  }

  private void configure() {
    m_absoluteEncoder = m_pivotMotor.getAbsoluteEncoder();
    m_pivotPIDController = m_pivotMotor.getClosedLoopController();
    ClosedLoopConfig pivotPIDConfig = new ClosedLoopConfig().pidf(PIVOT.PIVOT_P, PIVOT.PIVOT_I, PIVOT.PIVOT_D, PIVOT.PIVOT_FF).outputRange(-1,1).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    m_pivotMotor.setInverted(PIVOT.MOTOR_INVERTED);

    AbsoluteEncoderConfig pivotAbsoluteEncoderConfig = new AbsoluteEncoderConfig().inverted(PIVOT.ENCODER_INVERTED).positionConversionFactor(PIVOT.ENCODER_POSITION_CONVERSION_FACTOR).velocityConversionFactor(PIVOT.ENCODER_VELOCITY_CONVERSION_FACTOR);

    SparkBaseConfig pivotBaseConfig = new SparkMaxConfig()
      .inverted(PIVOT.MOTOR_INVERTED)
      .idleMode(PIVOT.IDLE_MODE)
      .smartCurrentLimit(PIVOT.MOTOR_STALL_LIMIT_AMPS, PIVOT.MOTOR_FREE_LIMIT_AMPS)
      .voltageCompensation(12)
      .apply(pivotPIDConfig)
      .apply(pivotAbsoluteEncoderConfig);
    
    m_pivotMotor.configure(pivotBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public boolean isSettingAngle() {
    return !Double.isNaN(m_targetAngle);
  }

  public void setAngle(double angle) {
    if (Double.isNaN(angle)) {
      System.err.println("PIVOT: Cannot set angle to NaN!");
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
    // double currentOffset = m_absoluteEncoder.getZeroOffset();
    // double newOffset = getCurrentAngle() + currentOffset - Constants.PIVOT.MIN_PIVOT_ANGLE;
    // newOffset %= 360;
    // setZeroOffset(newOffset);
    System.out.println("[Pivot] Zero Not Set. Set the zero manually");
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Pivot angle", getCurrentAngle());
    // SmartDashboard.putNumber("Pivot target angle", getTargetAngle());
  }
}
