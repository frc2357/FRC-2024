package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.DIGITAL_INPUT;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.LEDs.LEDsSetColor;
import frc.robot.Robot;
import frc.robot.state.RobotState.NoteState;

public class Intake extends SubsystemBase {
  private CANSparkMax m_topIntakeMotor;
  private CANSparkMax m_bottomIntakeMotor;

  private DigitalInput m_beamBreakSensor;

  public Intake() {
    m_topIntakeMotor = new CANSparkMax(CAN_ID.TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
    m_bottomIntakeMotor = new CANSparkMax(CAN_ID.BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);

    m_beamBreakSensor = new DigitalInput(DIGITAL_INPUT.INTAKE_BEAM_BREAK_ID);

    configure();
  }

  public void configure() {
    m_topIntakeMotor.setInverted(INTAKE.TOP_MOTOR_INVERTED);
    m_bottomIntakeMotor.setInverted(INTAKE.BOTTOM_MOTOR_INVERTED);

    m_topIntakeMotor.enableVoltageCompensation(12);
    m_topIntakeMotor.setIdleMode(INTAKE.IDLE_MODE);
    m_topIntakeMotor.setSmartCurrentLimit(
        INTAKE.TOP_MOTOR_STALL_LIMIT_AMPS, INTAKE.TOP_MOTOR_FREE_LIMIT_AMPS);

    m_bottomIntakeMotor.enableVoltageCompensation(12);
    m_bottomIntakeMotor.setIdleMode(INTAKE.IDLE_MODE);
    m_bottomIntakeMotor.setSmartCurrentLimit(
        INTAKE.BOTTOM_MOTOR_STALL_LIMIT_AMPS, INTAKE.BOTTOM_MOTOR_FREE_LIMIT_AMPS);
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
    return !m_beamBreakSensor.get();
  }

  public void stop() {
    m_topIntakeMotor.set(0);
    m_bottomIntakeMotor.set(0);
  }

  @Override
  public void periodic() {
    if (Robot.state.isNote(NoteState.NOTE_STOWED) && !isBeamBroken()) {
      new LEDsSetColor(LEDs.MELTDOWN_ORANGE).schedule();
      Robot.state.setNoteState(NoteState.EMPTY);
    }
  }
}
