package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterPivotSetSetpoint extends Command {
  private double m_setpoint;

  public ShooterPivotSetSetpoint(double setpoint) {
    m_setpoint = setpoint;
    addRequirements(Robot.pivot);
  }

  @Override
  public void initialize() {
    Robot.pivot.setPivotRotations(m_setpoint);
  }

  @Override
  public boolean isFinished() {
    return Robot.pivot.isPivotAtRotations();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.pivot.stop();
  }
}
