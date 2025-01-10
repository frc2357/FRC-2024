package frc.robot.commands.extensionArm;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ExtensionArmMoveToDistance extends Command {
  private Distance m_distance;

  public ExtensionArmMoveToDistance(Distance distance) {
    m_distance = distance;
    addRequirements(Robot.extensionArm);
  }

  @Override
  public void initialize() {
    Robot.extensionArm.setExtensionDistance(m_distance);
  }

  @Override
  public boolean isFinished() {
    return Robot.extensionArm.isAtTargetRotations();
  }

  @Override
  public void end(boolean interupted) {
    Robot.extensionArm.stop();
  }
}
