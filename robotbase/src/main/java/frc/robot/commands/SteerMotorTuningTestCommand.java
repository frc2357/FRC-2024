package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SteerMotorTuningTestCommand extends Command {
  // public double m_total = 0;
  // public long m_i = 0;
  // public double m_setSpeed = 2;

  public SteerMotorTuningTestCommand() {
    addRequirements(Robot.drive);
  }

  @Override
  public void initialize() {
    Robot.drive.drive(1, 0, 0);
  }

  @Override
  public void execute() {
    // double speed = Robot.drive.getModuleStates()[0].speedMetersPerSecond;
    // if (-0.95 * m_setSpeed > speed) {
    // m_i++;
    // m_total += speed;
    // }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.drive.drive(0, 1, 0);
    // System.out.println("*****************************************************");
    // System.out.println(m_total / m_i);
    // System.out.println("*****************************************************");
  }
}
