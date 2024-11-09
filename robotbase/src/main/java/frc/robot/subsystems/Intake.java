package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.DIGITAL_INPUT;
import frc.robot.Constants.INTAKE;

public class Intake extends SubsystemBase {
  private SparkMax m_topIntakeMotor;
  private SparkMax m_bottomIntakeMotor;

  private DigitalInput m_beamBreakSensor;

  private Debouncer m_debouncer;

  public Intake() {
    m_topIntakeMotor = new SparkMax(CAN_ID.TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
    m_bottomIntakeMotor = new SparkMax(CAN_ID.BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);

    m_beamBreakSensor = new DigitalInput(DIGITAL_INPUT.INTAKE_BEAM_BREAK_ID);

    m_debouncer = new Debouncer(INTAKE.DEBOUNCE_TIME_SECONDS, DebounceType.kBoth);

    configure();
  }

  public void configure() {

    SparkBaseConfig topMotorConfig = new SparkMaxConfig().inverted(INTAKE.TOP_MOTOR_INVERTED).voltageCompensation(12).idleMode(INTAKE.IDLE_MODE).smartCurrentLimit(INTAKE.TOP_MOTOR_STALL_LIMIT_AMPS, INTAKE.TOP_MOTOR_FREE_LIMIT_AMPS);
    SparkBaseConfig bottomMotorConfig = new SparkMaxConfig().inverted(INTAKE.BOTTOM_MOTOR_INVERTED).voltageCompensation(12).idleMode(INTAKE.IDLE_MODE).smartCurrentLimit(INTAKE.BOTTOM_MOTOR_STALL_LIMIT_AMPS, INTAKE.BOTTOM_MOTOR_FREE_LIMIT_AMPS);

    m_topIntakeMotor.configure(topMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_bottomIntakeMotor.configure(bottomMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void set(double percentOutput) {
    m_topIntakeMotor.set(percentOutput);
    m_bottomIntakeMotor.set(percentOutput);
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= INTAKE.AXIS_MAX_SPEED;
    set(axisSpeed);
  }

  public boolean isBeamBroken() {
    return m_debouncer.calculate(!m_beamBreakSensor.get());
  }

  public void stop() {
    m_topIntakeMotor.set(0);
    m_bottomIntakeMotor.set(0);
  }

  public double getTopCurrent() {
    return m_topIntakeMotor.getOutputCurrent();
  }

  public double getBottomCurrent() {
    return m_bottomIntakeMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(
        "Top Intake Running", Math.abs(m_topIntakeMotor.getEncoder().getVelocity()) >= 50);
    SmartDashboard.putBoolean(
        "Bottom Intake Running", Math.abs(m_bottomIntakeMotor.getEncoder().getVelocity()) >= 50);
  }
}
