package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class TargetLockOnNote extends Command {
  private int m_startingPipeline;

  public TargetLockOnNote() {
    m_startingPipeline = Robot.intakeCam.getPipeline();
    addRequirements(Robot.swerve, Robot.intakeCam);
  }

  @Override
  public void initialize() {
    Robot.intakeCam.setNeuralNetworkPipelineActive();
  }

  @Override
  public void execute() {
    var targetYaw = Robot.intakeCam.getNoteTargetYaw();

    Robot.swerve.driveTargetLock(
        Robot.driverControls.getY() * SWERVE.MAX_SPEED_METERS_PER_SECOND,
        Robot.driverControls.getX() * SWERVE.MAX_SPEED_METERS_PER_SECOND,
        !Double.isNaN(targetYaw) ? targetYaw : 0,
        !Double.isNaN(targetYaw));
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
    Robot.intakeCam.setPipeline(m_startingPipeline);
  }
}
