package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.END_AFFECTOR;

public class EndAffectorSubsystem extends SubsystemBase {

  private CANSparkMax m_motor;

  public EndAffectorSubsystem() {
    m_motor = new CANSparkMax(CAN_ID.END_AFFECTOR_MOTOR_ID, MotorType.kBrushed);
    configure();
  }

  private void configure(){
    m_motor.setInverted(END_AFFECTOR.IS_INVERTED);
    m_motor.setIdleMode(END_AFFECTOR.IDLE_MODE);
    m_motor.setSmartCurrentLimit(END_AFFECTOR.MOTOR_STALL_LIMIT_AMPS, END_AFFECTOR.MOTOR_FREE_LIMIT_AMPS);
    m_motor.enableVoltageCompensation(12);
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



  public void getMotorSpeed() {
    m_motor.get();
  }
}
