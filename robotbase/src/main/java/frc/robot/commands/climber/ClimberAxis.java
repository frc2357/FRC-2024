package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ClimberAxis extends Command {
  private AxisInterface m_xAxis;
  private AxisInterface m_yAxis;

  public ClimberAxis(AxisInterface xAxis, AxisInterface yAxis) {
    m_xAxis = xAxis;
    m_yAxis = yAxis;
    addRequirements(Robot.climber);
  }

  @Override
  public void execute() {
    double xAxisSpeed = m_xAxis.getValue();
    double yAxisSpeed = m_yAxis.getValue();

    double leftSpeed = (-xAxisSpeed + yAxisSpeed) / 2.0;
    double rightSpeed = (xAxisSpeed + yAxisSpeed) / 2.0;

    Robot.climber.setSpeed(leftSpeed, rightSpeed);
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
