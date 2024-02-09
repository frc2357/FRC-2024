package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax m_topIntakeMotor;
  private CANSparkMax m_bottomIntakeMotor;
  private DigitalInput m_beamBreakSensor;

  public Intake() {
    m_topIntakeMotor = new CANSparkMax(Constants.CAN_ID.TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
    m_bottomIntakeMotor =
        new CANSparkMax(Constants.CAN_ID.BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);

    m_beamBreakSensor = new DigitalInput(Constants.DIGITAL_INPUT.INTAKE_BEAM_BREAK_ID);
  }

  public void configure() {
    m_topIntakeMotor.setInverted(Constants.INTAKE.TOP_MOTOR_INVERTED);
    m_bottomIntakeMotor.setInverted(Constants.INTAKE.BOTTOM_MOTOR_INVERTED);

    m_topIntakeMotor.enableVoltageCompensation(12);
    m_topIntakeMotor.setIdleMode(Constants.INTAKE.IDLE_MODE);
    m_topIntakeMotor.setSmartCurrentLimit(
        Constants.INTAKE.TOP_MOTOR_STALL_LIMIT_AMPS, Constants.INTAKE.TOP_MOTOR_FREE_LIMIT_AMPS);

    m_bottomIntakeMotor.enableVoltageCompensation(12);
    m_bottomIntakeMotor.setIdleMode(Constants.INTAKE.IDLE_MODE);
    m_bottomIntakeMotor.setSmartCurrentLimit(
        Constants.INTAKE.BOTTOM_MOTOR_STALL_LIMIT_AMPS,
        Constants.INTAKE.BOTTOM_MOTOR_FREE_LIMIT_AMPS);
  }

  public void set(double topPO, double bottomPO) {
    m_topIntakeMotor.set(topPO);
    m_bottomIntakeMotor.set(bottomPO);
  }

  public void setAxisSpeed(double topAxisSpeed, double bottomAxisSpeed) {
    topAxisSpeed *= Constants.INTAKE.AXIS_MAX_SPEED;
    bottomAxisSpeed *= Constants.INTAKE.AXIS_MAX_SPEED;
    set(topAxisSpeed,bottomAxisSpeed);
  }
  

  public boolean isBeamBroken() {
    return m_beamBreakSensor.get();
  }

  public void stop() {
    m_topIntakeMotor.set(0);
    m_bottomIntakeMotor.set(0);
  }
}
