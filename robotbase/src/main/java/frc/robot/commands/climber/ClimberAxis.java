package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ClimberAxis extends Command {
  private AxisInterface m_axis;

  public ClimberAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.climber);
  }

  @Override
  public void execute() {
    double axisSpeed = m_axis.getValue();
    Robot.climber.set(axisSpeed, axisSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climber.stop();
  }
}
