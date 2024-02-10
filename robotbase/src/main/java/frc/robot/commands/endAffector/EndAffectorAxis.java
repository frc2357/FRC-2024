package frc.robot.commands.endAffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class EndAffectorAxis extends Command {
  private AxisInterface m_axis;

  public EndAffectorAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.endAffector);
  }

  @Override
  public void execute() {
    double axisSpeed = m_axis.getValue();
    Robot.endAffector.setAxisSpeed(axisSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.endAffector.stop();
  }
}
