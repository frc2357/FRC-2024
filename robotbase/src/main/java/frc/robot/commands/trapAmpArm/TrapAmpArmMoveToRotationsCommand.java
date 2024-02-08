package frc.robot.commands.trapAmpArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TrapAmpArmMoveToRotationsCommand extends Command {
  private double m_rotations;

  public TrapAmpArmMoveToRotationsCommand(double rotations) {
    m_rotations = rotations;
    addRequirements(Robot.trapAmpArm);
  }

  @Override
  public void initialize() {
    Robot.trapAmpArm.setTargetRotations(m_rotations);
  }

  @Override
  public boolean isFinished() {
    return Robot.trapAmpArm.isAtTargetRotations();
  }

  @Override
  public void end(boolean interupted) {
    Robot.trapAmpArm.stop();
  }
}
