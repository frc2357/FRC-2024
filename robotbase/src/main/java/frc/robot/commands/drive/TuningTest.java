package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TuningTest extends Command {

  private double m_setpoint;
  private int m_loopNum;

  public TuningTest(double setPoint) {
    addRequirements(Robot.swerve);
    m_setpoint = setPoint;
  }

  @Override
  public void execute() {
    if (m_loopNum++ % 10 == 0) {
      Robot.swerve.drive(m_setpoint, 0, 0);
      System.out.println(
          "SETPOINT: "
              + m_setpoint
              + "\nROBOT SPEED: "
              + Robot.swerve.getModuleStates()[0].speedMetersPerSecond);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interupted) {
    Robot.swerve.drive(0, 0, 0);
    System.out.println(
        "SETPOINT: "
            + m_setpoint
            + "\nROBOT VELOCITY: "
            + Robot.swerve.getModuleStates()[0].speedMetersPerSecond);
  }
}
