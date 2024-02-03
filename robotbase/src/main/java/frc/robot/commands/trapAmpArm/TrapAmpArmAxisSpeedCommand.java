package frc.robot.commands.trapAmpArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class TrapAmpArmAxisSpeedCommand extends Command {
  private AxisInterface m_axis;

  public TrapAmpArmAxisSpeedCommand(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.trapAmpArm);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    Robot.trapAmpArm.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.trapAmpArm.stop();
  }
}
