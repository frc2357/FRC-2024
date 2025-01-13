package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.util.Utility;

public class ExtensionArm extends SubsystemBase {
  private Angle m_targetRotations = Units.Rotations.of(Double.NaN);

  private SparkMax m_motor;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  public ExtensionArm() {
    m_motor = new SparkMax(Constants.CAN_ID.TRAP_AMP_ARM_MOTOR_ID, MotorType.kBrushless);
    m_PIDController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getAlternateEncoder();

    configure();
  }

  private void configure() {
    MAXMotionConfig maxMotionConfig = new MAXMotionConfig()
        .maxVelocity(EXTENSION_ARM.SMART_MOTION_MAX_VEL_RPM)
        .maxAcceleration(EXTENSION_ARM.SMART_MOTION_MAX_ACC_RPM)
        .allowedClosedLoopError(EXTENSION_ARM.SMART_MOTION_ALLOWED_ERROR);

    SparkMaxConfig motorConfig = new SparkMaxConfig();

    AlternateEncoderConfig encoderConfig = new AlternateEncoderConfig().inverted(EXTENSION_ARM.ENCODER_INVERTED)
        .countsPerRevolution(8196);

    motorConfig.closedLoop
        .pidf(EXTENSION_ARM.MOTOR_PID_P, EXTENSION_ARM.MOTOR_PID_I,
            EXTENSION_ARM.MOTOR_PID_D,
            EXTENSION_ARM.MOTOR_PID_FF)
        .outputRange(-1,
            1)
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder).apply(maxMotionConfig);

    motorConfig.apply(encoderConfig);

    motorConfig.inverted(EXTENSION_ARM.MOTOR_IS_INVERTED).idleMode(EXTENSION_ARM.MOTOR_IDLE_MODE)
        .smartCurrentLimit(EXTENSION_ARM.MOTOR_STALL_LIMIT_AMPS, EXTENSION_ARM.MOTOR_FREE_LIMIT_AMPS);

    m_motor.configure(motorConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

  }

  private void setTargetRotations(Angle targetRotations) {
    m_targetRotations = targetRotations;
    m_PIDController.setReference(targetRotations.in(Units.Rotations), ControlType.kMAXMotionPositionControl);
  }

  public void setAxisSpeed(double speed) {
    m_targetRotations = Units.Rotations.of(Double.NaN);
    speed *= EXTENSION_ARM.AXIS_MAX_SPEED;
    m_motor.set(speed);
  }

  public void stop() {
    m_targetRotations = Units.Rotations.of(Double.NaN);
    m_motor.stopMotor();
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  public boolean isAtTargetRotations() {
    return Utility.isWithinTolerance(
        getRotations().in(Units.Rotations),
        m_targetRotations.in(Units.Rotations),
        EXTENSION_ARM.SMART_MOTION_ALLOWED_ERROR);
  }

  public Angle getRotations() {
    return Units.Rotations.of(m_encoder.getPosition());
  }

  public Distance getExtensionDistance() {
    return EXTENSION_ARM.MOTOR_PULLEY_PITCH_CIRCUMFERENCE.times(getRotations().in(Units.Rotations));
  }

  public void setExtensionDistance(Distance distance) {
    Angle rotations = Units.Rotations
        .of(distance.div(EXTENSION_ARM.MOTOR_PULLEY_PITCH_CIRCUMFERENCE).magnitude());
    System.out.println(rotations.in(Units.Rotations));
    setExtensionRotations(rotations);
  }

  public void setExtensionRotations(Angle rotations) {
    System.out.println(rotations.in(Units.Rotations));
    setTargetRotations(rotations);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Extension arm distance (inches)", getExtensionDistance().in(Units.Inches));
    SmartDashboard.putNumber("Extension arm rotations (revs)", getRotations().in(Units.Rotations));
  }
}
