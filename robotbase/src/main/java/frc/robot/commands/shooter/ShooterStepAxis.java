package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ShooterStepAxis extends Command {
  private AxisInterface m_axis;
  private double m_step;

  /**
   * Same as ShooterRollerAxisCommand but rounds to the nearest step
   *
   * @param axis Controller axis
   * @param step value from 0 to 1
   */
  public ShooterStepAxis(AxisInterface axis, double step) {
    m_axis = axis;
    m_step = step;
    addRequirements(Robot.shooter);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    axisValue = Math.floor(axisValue / m_step) * m_step;
    Robot.shooter.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stop();
  }
}
