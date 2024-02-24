package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DriveAtSpeed extends Command {
   private double m_xSpeed;
   private double m_ySpeed;
   private double m_timeSeconds;
   private long m_startTimeMillis;

   public DriveAtSpeed(double xSpeed, double ySpeed, double timeSeconds) {
      m_xSpeed = xSpeed;
      m_ySpeed = ySpeed;
      m_timeSeconds = timeSeconds;
      addRequirements(Robot.swerve);
   }

   @Override
   public void initialize() {
      m_startTimeMillis = System.currentTimeMillis();
      Robot.swerve.drive(m_xSpeed, m_ySpeed, 0);
   }

   @Override
   public boolean isFinished() {
      return (m_timeSeconds != -1) && (System.currentTimeMillis() >= m_startTimeMillis + (1000 * m_timeSeconds));
   }

   @Override
   public void end(boolean interrupted) {
      Robot.swerve.drive(0, 0, 0);
   }
}
