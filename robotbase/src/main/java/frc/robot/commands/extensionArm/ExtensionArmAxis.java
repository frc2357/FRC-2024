package frc.robot.commands.extensionArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ExtensionArmAxis extends Command {
  private AxisInterface m_axis;

  public ExtensionArmAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.extensionArm);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    double motorSpeed = (-axisValue) * EXTENSION_ARM.AXIS_MAX_SPEED;
    Robot.extensionArm.setSpeed(motorSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.extensionArm.stop();
  }
}
