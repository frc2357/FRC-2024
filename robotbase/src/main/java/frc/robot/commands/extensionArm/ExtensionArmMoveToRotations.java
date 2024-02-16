package frc.robot.commands.extensionArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ExtensionArmMoveToRotations extends Command {
  private double m_rotations;

  public ExtensionArmMoveToRotations(double rotations) {
    m_rotations = rotations;
    addRequirements(Robot.extensionArm);
  }

  @Override
  public void initialize() {
    Robot.extensionArm.setTargetRotations(m_rotations);
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
