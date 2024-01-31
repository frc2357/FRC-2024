package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.END_AFFECTOR;

public class EndAffectorSubsystem extends SubsystemBase {

  private Talon m_motor;

  public EndAffectorSubsystem() {
    m_motor = new Talon(CAN_ID.END_AFFECTOR_MOTOR_ID);
    m_motor.setInverted(END_AFFECTOR.IS_INVERTED);
    m_motor.enableDeadbandElimination(END_AFFECTOR.ELIMINATE_DEADBAND);
    m_motor.setSafetyEnabled(END_AFFECTOR.IS_MOTOR_SAFTEY_ENFORCED);
  }

  public void setSpeed(double speedPercentage) {
    m_motor.set(speedPercentage);
  }

  public void setVoltage(double outputVoltage) {
    m_motor.setVoltage(outputVoltage);
  }

  public void stopMotor() {
    m_motor.stopMotor();
  }

  public void feedMotorSafety() {
    m_motor.feed();
  }

  public void checkMotor() {
    m_motor.check();
  }

  public void getMotorSpeed() {
    m_motor.get();
  }
}
