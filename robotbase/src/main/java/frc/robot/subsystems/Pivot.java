package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIVOT;
import frc.robot.Robot;
import frc.robot.util.RobotMath;
import frc.robot.util.Utility;

public class Pivot extends SubsystemBase {
  private boolean m_isClosedLoopEnabled = false;
  private boolean m_isVisionTargeting = false;

  private CANSparkMax m_pivotMotor;
  private SparkPIDController m_pivotPIDController;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private double m_targetRotation;

  public Pivot() {
    m_pivotMotor = new CANSparkMax(Constants.CAN_ID.PIVOT_MOTOR_ID, MotorType.kBrushless);
    configure();
  }

  private void configure() {
    m_pivotMotor.setInverted(PIVOT.MOTOR_INVERTED);
    m_pivotMotor.setIdleMode(PIVOT.IDLE_MODE);
    m_pivotMotor.setSmartCurrentLimit(PIVOT.MOTOR_STALL_LIMIT_AMPS, PIVOT.MOTOR_FREE_LIMIT_AMPS);
    m_pivotMotor.enableVoltageCompensation(12);

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
    // m_absoluteEncoder.setZeroOffset(PIVOT.ENCODER_ZERO_OFFSET);

    // m_pivotPIDController.setPositionPIDWrappingEnabled(PIVOT.POSITION_PID_WRAPPING_ENABLED);
    m_pivotPIDController.setFeedbackDevice(m_absoluteEncoder);
  }

  public void setPivotRotation(double rotation) {
    m_isClosedLoopEnabled = true;
    m_targetRotation = rotation;
    System.out.println(rotation);
    m_pivotPIDController.setReference(m_targetRotation, ControlType.kPosition);
  }

  public void setPivotAxisSpeed(double axisSpeed) {
    m_isClosedLoopEnabled = false;
    double motorSpeed = (-axisSpeed) * PIVOT.AXIS_MAX_SPEED;
    m_pivotMotor.set(motorSpeed);
  }

  public void stop() {
    m_isClosedLoopEnabled = false;
    m_pivotMotor.stopMotor();
  }

  public void resetEncoder() {
    m_pivotMotor.getEncoder().setPosition(0);
    m_targetRotation = 0;
  }

  public double getPivotRotations() {
    return m_pivotMotor.getEncoder().getPosition();
  }

  public double getPosition() {
    return m_absoluteEncoder.getPosition();
  }

  public boolean isPivotAtRotation() {
    return Utility.isWithinTolerance(getPosition(), m_targetRotation, PIVOT.POSITION_ALLOWED_ERROR);
  }

  private boolean hasTarget() {
    return Robot.shooterCam.validTargetExists();
  }

  @Override
  public void periodic() {
    if (m_isClosedLoopEnabled) {
      if (isPivotAtRotation()) {
        stop();
      } else {
        m_pivotPIDController.setReference(m_targetRotation, ControlType.kPosition);
      }

      if (m_isVisionTargeting) {
        visionTargetingPeriodic();
      }
    }

    SmartDashboard.putNumber("Pivot Rotation", getPosition());
  }

  public void startVisionTargeting() {
    m_isVisionTargeting = true;
    m_isClosedLoopEnabled = true;
  }

  public void stopVisionTargeting() {
    m_isVisionTargeting = false;
    m_isClosedLoopEnabled = false;
    stop();
  }

  private void visionTargetingPeriodic() {
    if (hasTarget()) {
      setVisionTargetingRotation(Robot.shooterCam.getTY());
    } else {
      System.err.println("----- No vision target (Pivot) -----");
    }
  }

  private void setVisionTargetingRotation(double ty) {
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, ty);
    if (curveIndex == -1) {
      // System.err.println("----- Curve segment index out of bounds (Pivot) -----");
      return;
    }

    double[] high = Robot.shooterCurve[curveIndex];
    double[] low = Robot.shooterCurve[curveIndex + 1];

    double highTY = high[0];
    double lowTY = low[0];
    double highPivotRotation = high[1];
    double lowPivotRotation = low[1];

    double pivotRotation =
        RobotMath.linearlyInterpolate(highPivotRotation, lowPivotRotation, highTY, lowTY, ty);

    if (Double.isNaN(pivotRotation)) {
      // System.err.println("----- Invalid shooter pivot values -----");
      return;
    }

    setPivotRotation(pivotRotation);
  }
}