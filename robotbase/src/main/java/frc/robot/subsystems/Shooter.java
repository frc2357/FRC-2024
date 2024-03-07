package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SHOOTER;
import frc.robot.networkTables.ShooterCurveTuner;
import frc.robot.util.Utility;

public class Shooter extends SubsystemBase {
  private double m_targetRPM;

  private CANSparkMax m_topShooterMotor;
  private CANSparkMax m_bottomShooterMotor;

  private SparkPIDController m_topPIDController;
  private SparkPIDController m_bottomPIDController;

  private ShooterCurveTuner m_curveTuner;

  public Shooter() {
    m_topShooterMotor =
        new CANSparkMax(Constants.CAN_ID.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    m_bottomShooterMotor =
        new CANSparkMax(Constants.CAN_ID.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    m_curveTuner = new ShooterCurveTuner();

    m_targetRPM = Double.NaN;

    SmartDashboard.putNumber(SHOOTER.SHOOTER_OFFSET_KEY, 0.0);

    configure();
  }

  public void configure() {
    m_topShooterMotor.setInverted(Constants.SHOOTER.TOP_MOTOR_INVERTED);
    m_bottomShooterMotor.setInverted(Constants.SHOOTER.BOTTOM_MOTOR_INVERTED);

    m_topShooterMotor.setOpenLoopRampRate(Constants.SHOOTER.RAMP_RATE);
    m_topShooterMotor.enableVoltageCompensation(12);
    m_topShooterMotor.setIdleMode(Constants.SHOOTER.IDLE_MODE);
    m_topShooterMotor.setSmartCurrentLimit(
        Constants.SHOOTER.TOP_MOTOR_STALL_LIMIT_AMPS, Constants.SHOOTER.TOP_MOTOR_FREE_LIMIT_AMPS);

    m_bottomShooterMotor.setOpenLoopRampRate(Constants.SHOOTER.RAMP_RATE);
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

  public void setRPM(double topRPM) {
    if (Double.isNaN(topRPM)) {
      System.err.println("Shooter: Cannot set topRPM to NaN!");
      return;
    }

    m_targetRPM = topRPM;
    m_topPIDController.setReference(m_targetRPM, ControlType.kVelocity);
    m_bottomPIDController.setReference(m_targetRPM, ControlType.kVelocity);
  }

  public void setAxisSpeed(double speed) {
    m_targetRPM = Double.NaN;
    speed *= Constants.SHOOTER.SHOOTER_AXIS_MAX_SPEED;
    m_topShooterMotor.set(speed);
    m_bottomShooterMotor.set(speed);
  }

  public void stop() {
    m_targetRPM = Double.NaN;
    m_topShooterMotor.set(0.0);
    m_bottomShooterMotor.set(0.0);
  }

  public double getTopVelocity() {
    return m_topShooterMotor.getEncoder().getVelocity();
  }

  public double getBottomVelocity() {
    return m_bottomShooterMotor.getEncoder().getVelocity();
  }

  public boolean isAtRPM(double RPM) {
    return Utility.isWithinTolerance(getTopVelocity(), RPM, SHOOTER.RPM_TOLERANCE)
        && Utility.isWithinTolerance(getBottomVelocity(), RPM, SHOOTER.RPM_TOLERANCE);
  }

  public boolean isAtTargetSpeed() {
    return isAtRPM(m_targetRPM);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Top RPM", getTopVelocity());
    // SmartDashboard.putNumber("Bottom RPM", getBottomVelocity());
    // m_curveTuner.updateCurveValues();
  }

  public double[] getShooterCurveRow() {
    return m_curveTuner.getSelectedRow();
  }
}
