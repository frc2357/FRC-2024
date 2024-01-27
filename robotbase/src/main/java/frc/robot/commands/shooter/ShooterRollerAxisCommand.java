package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ShooterRollerAxisCommand extends Command {
  private AxisInterface m_axis;

  public ShooterRollerAxisCommand(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.shooter);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    Robot.shooter.setAxisSpeed(axisValue, axisValue);
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
