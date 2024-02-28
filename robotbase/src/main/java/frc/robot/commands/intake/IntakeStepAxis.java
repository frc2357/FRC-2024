package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class IntakeStepAxis extends Command {
  private AxisInterface m_axis;
  private double m_step;

  /**
   * Same as IntakeRollerAxisCommand but rounds to the nearest step
   *
   * @param axis Controller axis
   * @param step value from 0 to 1
   */
  public IntakeStepAxis(AxisInterface axis, double step) {
    m_axis = axis;
    m_step = step;
    addRequirements(Robot.intake);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    axisValue = Math.floor(axisValue / m_step) * m_step;
    Robot.intake.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
