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
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.DIGITAL_INPUT;
import frc.robot.Constants.END_AFFECTOR;

public class EndAffector extends SubsystemBase {

  private SparkMax m_motor;

  private DigitalInput m_proximitySensor;
  private DigitalOutput m_proximitySensorPower;

  private Debouncer m_debouncer;

  public EndAffector() {
    m_motor = new SparkMax(CAN_ID.END_AFFECTOR_MOTOR_ID, MotorType.kBrushed);

    m_proximitySensor = new DigitalInput(DIGITAL_INPUT.END_AFFECTOR_PROXIMITY_SENSOR_ID);
    m_proximitySensorPower =
        new DigitalOutput(DIGITAL_INPUT.END_AFFECTOR_PROXIMITY_SENSOR_POWER_ID);

    m_debouncer = new Debouncer(END_AFFECTOR.DEBOUNCE_TIME_SECONDS, DebounceType.kBoth);

    configure();
  }

  private void configure() {
    SparkBaseConfig motorConfig = new SparkMaxConfig().inverted(END_AFFECTOR.IS_INVERTED).idleMode(END_AFFECTOR.IDLE_MODE).smartCurrentLimit(END_AFFECTOR.MOTOR_STALL_LIMIT_AMPS, END_AFFECTOR.MOTOR_FREE_LIMIT_AMPS).voltageCompensation(12);
    m_motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setAxisSpeed(double axisSpeed) {
    double motorSpeed = (-axisSpeed) * END_AFFECTOR.AXIS_MAX_SPEED;
    m_motor.set(motorSpeed);
  }

  public void setSpeed(double speedPercentOut) {
    m_motor.set(speedPercentOut);
  }

  public void setVoltage(double outputVoltage) {
    m_motor.setVoltage(outputVoltage);
  }

  public boolean getProximitySensor() {
    // return m_debouncer.calculate(!m_proximitySensor.get());
    return !m_proximitySensor.get();
  }

  public void setProximitySensorPower(boolean on) {
    m_proximitySensorPower.set(on);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public double getMotorAmperage() {
    return m_motor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("End Affector Running", m_motor.get() >= 0.1);
  }
}
