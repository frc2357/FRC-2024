package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ClimberAxis extends Command {
  private AxisInterface m_rightAxis;
  private AxisInterface m_leftAxis;

  public ClimberAxis(AxisInterface rightAxis, AxisInterface leftAxis) {
    m_rightAxis = rightAxis;
    m_leftAxis = leftAxis;
    addRequirements(Robot.climber);
  }

  @Override
  public void execute() {
    double rightAxisSpeed = m_rightAxis.getValue();
    double leftAxisSpeed = m_leftAxis.getValue();
    Robot.climber.set(rightAxisSpeed, leftAxisSpeed);
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
