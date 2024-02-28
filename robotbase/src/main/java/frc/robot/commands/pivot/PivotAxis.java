package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class PivotAxis extends Command {
  private AxisInterface m_axis;

  public PivotAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.pivot);
  }

  @Override
  public void execute() {
    double axisSpeed = m_axis.getValue();
    double motorSpeed = (-axisSpeed) * Constants.PIVOT.AXIS_MAX_SPEED;
    Robot.pivot.setSpeed(motorSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.pivot.stop();
  }
}
