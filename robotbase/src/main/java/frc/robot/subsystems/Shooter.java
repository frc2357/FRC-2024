package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.RobotMath;
import frc.robot.util.Utility;

public class Shooter extends SubsystemBase {
  private boolean m_isVisionShooting = false;

  private double m_targetSpeedTop;
  private double m_targetSpeedBottom;

  private CANSparkMax m_topShooterMotor;
  private CANSparkMax m_bottomShooterMotor;

  private SparkPIDController m_topPIDController;
  private SparkPIDController m_bottomPIDController;

  private boolean m_isClosedLoopEnabled = false;

  public Shooter() {
    m_topShooterMotor =
        new CANSparkMax(Constants.CAN_ID.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    m_bottomShooterMotor =
        new CANSparkMax(Constants.CAN_ID.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    configure();
  }

  public void configure() {
    m_topShooterMotor.setInverted(Constants.SHOOTER.TOP_MOTOR_INVERTED);
    m_bottomShooterMotor.setInverted(Constants.SHOOTER.BOTTOM_MOTOR_INVERTED);

    m_topShooterMotor.enableVoltageCompensation(12);
    m_topShooterMotor.setIdleMode(Constants.SHOOTER.IDLE_MODE);
    m_topShooterMotor.setSmartCurrentLimit(
        Constants.SHOOTER.TOP_MOTOR_STALL_LIMIT_AMPS, Constants.SHOOTER.TOP_MOTOR_FREE_LIMIT_AMPS);

    m_bottomShooterMotor.enableVoltageCompensation(12);
    m_bottomShooterMotor.setIdleMode(Constants.SHOOTER.IDLE_MODE);
    m_bottomShooterMotor.setSmartCurrentLimit(
        Constants.SHOOTER.BOTTOM_MOTOR_STALL_LIMIT_AMPS,
        Constants.SHOOTER.BOTTOM_MOTOR_FREE_LIMIT_AMPS);

    m_topPIDController = m_topShooterMotor.getPIDController();
    m_bottomPIDController = m_bottomShooterMotor.getPIDController();

    m_topPIDController.setP(Constants.SHOOTER.TOP_MOTOR_P);
    m_topPIDController.setI(Constants.SHOOTER.TOP_MOTOR_I);
    m_topPIDController.setD(Constants.SHOOTER.TOP_MOTOR_D);
    m_topPIDController.setFF(Constants.SHOOTER.TOP_MOTOR_FF);

    m_bottomPIDController.setP(Constants.SHOOTER.BOTTOM_MOTOR_P);
    m_bottomPIDController.setI(Constants.SHOOTER.BOTTOM_MOTOR_I);
    m_bottomPIDController.setD(Constants.SHOOTER.BOTTOM_MOTOR_D);
    m_bottomPIDController.setFF(Constants.SHOOTER.BOTTOM_MOTOR_FF);

    m_topPIDController.setOutputRange(-1, 1);
    m_bottomPIDController.setOutputRange(-1, 1);
  }

  public void setRPMs(double topRPMS, double bottomRPMS) {
    m_targetSpeedTop = topRPMS;
    m_targetSpeedBottom = bottomRPMS;
    m_topPIDController.setReference(m_targetSpeedTop, ControlType.kVelocity);
    m_bottomPIDController.setReference(m_targetSpeedBottom, ControlType.kVelocity);
  }

  public void setAxisSpeed(double top, double bottom) {
    m_isClosedLoopEnabled = false;
    m_topShooterMotor.set(top);
    m_bottomShooterMotor.set(bottom);
  }

  public void stop() {
    m_isClosedLoopEnabled = false;
    m_topShooterMotor.set(0.0);
    m_bottomShooterMotor.set(0.0);
  }

  public double getTopVelocity() {
    return m_topShooterMotor.getEncoder().getVelocity();
  }

  public double getBottomVelocity() {
    return m_bottomShooterMotor.getEncoder().getVelocity();
  }

  public boolean isAtRPMs(double topRPMs, double bottomRPMs) {
    return Utility.isWithinTolerance(getTopVelocity(), topRPMs, 100)
        && Utility.isWithinTolerance(getBottomVelocity(), bottomRPMs, 100);
  }

  public boolean isAtTargetSpeed() {
    return isAtRPMs(m_targetSpeedTop, m_targetSpeedBottom);
  }

  private boolean hasTarget() {
    return Robot.shooterCam.validTargetExists();
  }

  @Override
  public void periodic() {
    if (m_isClosedLoopEnabled) {
      visionShotPeriodic();
    }
  }

  public void startVisionShooting() {
    m_isClosedLoopEnabled = true;
    m_isVisionShooting = true;
  }

  public void stopVisionShooting() {
    m_isClosedLoopEnabled = false;
    m_isVisionShooting = false;
    stop();
  }

  private void visionShotPeriodic() {
    if (hasTarget()) {
      setVisionShotRPMs(Robot.shooterCam.getTY());
    } else {
      System.err.println("----- No vision target (Shooter) -----");
    }
  }

  private void setVisionShotRPMs(double ty) {
    int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, ty);
    if (curveIndex == -1) {
      // System.err.println("----- Curve segment index out of bounds (Shooter)
      // -----");
      return;
    }

    double[] high = Robot.shooterCurve[curveIndex];
    double[] low = Robot.shooterCurve[curveIndex + 1];

    double highTY = high[0];
    double lowTY = low[0];
    double highBottomRPMs = high[2];
    double lowBottomRPMs = low[2];
    double highTopRPMs = high[3];
    double lowTopRPMs = low[3];

    double topRPMs = RobotMath.linearlyInterpolate(highTopRPMs, lowTopRPMs, highTY, lowTY, ty);
    double bottomRPMs =
        RobotMath.linearlyInterpolate(highBottomRPMs, lowBottomRPMs, highTY, lowTY, ty);

    if (Double.isNaN(topRPMs) || Double.isNaN(bottomRPMs)) {
      System.err.println("----- Invalid shooter values -----");
      return;
    }

    setRPMs(topRPMs, bottomRPMs);
  }
}
