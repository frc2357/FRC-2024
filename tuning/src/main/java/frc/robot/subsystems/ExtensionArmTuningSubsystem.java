package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ExtensionArmTuningSubsystem {
  private CANSparkMax m_motor;

  private SparkPIDController m_PID;
  private RelativeEncoder m_encoder;

  private double kP = 0.001;
  private double kI = 0;
  private double kD = 0;
  private double kFF = 0;
  private double maxVel = 2000;// 5700; // These are the new rpms, not rev throughbore encoder rpms
  private double maxAcc = 1000; // 5700;

  private double axisMaxSpeed = 0.5;

  public ExtensionArmTuningSubsystem() {
    m_motor = new CANSparkMax(31, MotorType.kBrushless);

    m_PID = m_motor.getPIDController();
    m_encoder = m_motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

    configure();
    display();
  }

  public void configure() {
    m_motor.setInverted(false);

    m_motor.enableVoltageCompensation(12);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(40, 40);

    m_encoder.setPositionConversionFactor(1);
    m_encoder.setVelocityConversionFactor(1);
    m_encoder.setInverted(true);

    m_PID.setOutputRange(-1, 1);

    m_PID.setFeedbackDevice(m_encoder);

    m_PID.setSmartMotionMaxVelocity(maxVel, 0);
    m_PID.setSmartMotionMinOutputVelocity(0, 0);
    m_PID.setSmartMotionMaxAccel(maxAcc, 0);
    m_PID.setSmartMotionAllowedClosedLoopError(0.1, 0);

    updatePIDs();
  }

  public void updatePIDs() {
    m_PID.setP(kP);
    m_PID.setI(kI);
    m_PID.setD(kD);
    m_PID.setFF(kFF);
    m_PID.setSmartMotionMaxVelocity(maxVel, 0);
    m_PID.setSmartMotionMaxAccel(maxAcc, 0);
  }

  public void display() {
    SmartDashboard.putNumber("Arm P", kP);
    SmartDashboard.putNumber("Arm I", kI);
    SmartDashboard.putNumber("Arm D", kD);
    SmartDashboard.putNumber("Arm FF", kFF);
    SmartDashboard.putNumber("Arm MaxVel", maxVel);
    SmartDashboard.putNumber("Arm MaxAcc", maxAcc);
    // This is the REV Throughbore position setpoint
    SmartDashboard.putNumber("Arm Setpoint", 0);
  }

  public void update() {
    kP = SmartDashboard.getNumber("Arm P", kP);
    kI = SmartDashboard.getNumber("Arm I", kI);
    kD = SmartDashboard.getNumber("Arm D", kD);
    kFF = SmartDashboard.getNumber("Arm FF", kFF);
    maxVel = SmartDashboard.getNumber("Arm MaxVel", maxVel);
    maxAcc = SmartDashboard.getNumber("Arm MaxAcc", maxAcc);

    updatePIDs();

    double motorRPMs, encoderRPMs;
    motorRPMs = m_motor.getEncoder().getVelocity();
    encoderRPMs = m_encoder.getVelocity();

    SmartDashboard.putNumber("Arm RPMs", motorRPMs);
    SmartDashboard.putNumber("Encoder RPMs", encoderRPMs);

    double motorPos, encoderPos;
    motorPos = m_motor.getEncoder().getPosition();
    encoderPos = m_encoder.getPosition();

    SmartDashboard.putNumber("Arm Pos", motorPos);
    SmartDashboard.putNumber("Encoder Pos", encoderPos);
  }

  public void teleopPeriodic() {
    double rotationSetpoint;
    rotationSetpoint = SmartDashboard.getNumber("Arm Setpoint", 0);
    System.out.println(rotationSetpoint);
    m_PID.setReference(rotationSetpoint, CANSparkMax.ControlType.kSmartMotion);
  }

  public void axisRun(double speed) {
    m_motor.set(speed * axisMaxSpeed);
  }

  public void resetEncoders() {
    m_motor.getEncoder().setPosition(0);
    m_encoder.setPosition(0);
  }
}
