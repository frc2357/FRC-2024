package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TuningTest1 extends Command {

  private double m_setpoint;
  private int m_loopNum;

  public TuningTest1(double setPoint) {
    addRequirements(Robot.swerve);
    m_setpoint = setPoint;
  }

  @Override
  public void execute() {
    if (m_loopNum++ % 5 == 0) {
      Robot.swerve.drive(0, m_setpoint, 0);
      System.out.println(
          "SETPOINT: "
              + 90
              + "\nSTEER ANGLE: "
              + Robot.swerve.getModuleStates()[0].angle.getDegrees());
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
