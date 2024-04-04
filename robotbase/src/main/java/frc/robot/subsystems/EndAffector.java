package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.DIGITAL_INPUT;
import frc.robot.Constants.END_AFFECTOR;

public class EndAffector extends SubsystemBase {

  private CANSparkMax m_motor;

  private DigitalInput m_proximitySensor;
  private DigitalOutput m_proximitySensorPower;

  public EndAffector() {
    m_motor = new CANSparkMax(CAN_ID.END_AFFECTOR_MOTOR_ID, MotorType.kBrushed);

    m_proximitySensor = new DigitalInput(DIGITAL_INPUT.END_AFFECTOR_PROXIMITY_SENSOR_ID);
    m_proximitySensorPower =
        new DigitalOutput(DIGITAL_INPUT.END_AFFECTOR_PROXIMITY_SENSOR_POWER_ID);

    configure();
  }

  private void configure() {
    m_motor.setInverted(END_AFFECTOR.IS_INVERTED);
    m_motor.setIdleMode(END_AFFECTOR.IDLE_MODE);
    m_motor.setSmartCurrentLimit(
        END_AFFECTOR.MOTOR_STALL_LIMIT_AMPS, END_AFFECTOR.MOTOR_FREE_LIMIT_AMPS);
    m_motor.enableVoltageCompensation(12);
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
    return !m_proximitySensor.get();
  }

  public void setProximitySensorPower(boolean on) {
    m_proximitySensorPower.set(on);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public void getMotorSpeed() {
    m_motor.get();
  }

  public double getMotorAmperage() {
    return m_motor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Prox", getProximitySensor());
  }
}
