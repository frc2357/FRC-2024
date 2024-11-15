package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.util.Utility;

public class ExtensionArm extends SubsystemBase {
  private SparkMax m_motor;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;
  private double m_targetRotations;

  public ExtensionArm() {
    m_motor = new SparkMax(Constants.CAN_ID.TRAP_AMP_ARM_MOTOR_ID, MotorType.kBrushless);
    configure();
  }

  private void configure() {
    MAXMotionConfig maxMotionConfig = new MAXMotionConfig().maxVelocity(EXTENSION_ARM.SMART_MOTION_MAX_VEL_RPM).maxAcceleration(EXTENSION_ARM.SMART_MOTION_MAX_ACC_RPM).allowedClosedLoopError(EXTENSION_ARM.SMART_MOTION_ALLOWED_ERROR);
    
    ClosedLoopConfig PIDConfig = new ClosedLoopConfig().pidf(EXTENSION_ARM.MOTOR_PID_P, EXTENSION_ARM.MOTOR_PID_I, EXTENSION_ARM.MOTOR_PID_D, EXTENSION_ARM.MOTOR_PID_FF)
    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder).outputRange(-1, 1).apply(maxMotionConfig);
    
    AlternateEncoderConfig altEncoderConfig = new AlternateEncoderConfig().inverted(EXTENSION_ARM.ENCODER_INVERTED);
    
    SparkBaseConfig motorConfig = new SparkMaxConfig().inverted(EXTENSION_ARM.MOTOR_IS_INVERTED).idleMode(EXTENSION_ARM.MOTOR_IDLE_MODE).smartCurrentLimit(EXTENSION_ARM.MOTOR_STALL_LIMIT_AMPS, EXTENSION_ARM.MOTOR_FREE_LIMIT_AMPS).apply(PIDConfig);
    
    m_motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_encoder = m_motor.getAlternateEncoder();

    m_PIDController = m_motor.getClosedLoopController();
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
    m_targetRotations = Double.NaN;
  }

  public void setTargetRotations(double targetRotations) {
    m_targetRotations = targetRotations;
    m_PIDController.setReference(m_targetRotations, ControlType.kMAXMotionPositionControl);
  }

  public void stop() {
    m_motor.stopMotor();
    m_targetRotations = Double.NaN;
  }

  public void setZero() {
    m_encoder.setPosition(0);
    System.out.println("[Extension Arm] Zero Set");
  }

  public double getRotations() {
    return m_encoder.getPosition() * -1; // DO NOT CHANGE! rev is stupid and we cant invert the alt encoder
  }

  public double getVelocity() {
    return m_encoder.getVelocity() * -1; // DO NOT CHANGE! rev is stupid and we cant invert the alt encoder
  }

  public boolean isAtTargetRotations() {
    return Utility.isWithinTolerance(
        getRotations(), m_targetRotations, EXTENSION_ARM.SMART_MOTION_ALLOWED_ERROR);
  }
}
