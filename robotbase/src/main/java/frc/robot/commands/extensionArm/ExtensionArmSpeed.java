package frc.robot.commands.extensionArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ExtensionArmSpeed extends Command {
  private double m_speed;

  public ExtensionArmSpeed(double speed) {
    m_speed = speed;
    addRequirements(Robot.extensionArm);
  }

  @Override
  public void execute() {
    Robot.extensionArm.setSpeed(m_speed);
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
