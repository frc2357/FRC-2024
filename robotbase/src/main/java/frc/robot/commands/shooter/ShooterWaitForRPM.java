package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterWaitForRPM extends Command {
  @Override
  public boolean isFinished() {
    var isAtTargetSpeed = Robot.shooter.isAtTargetSpeed();
    System.out.println("[ShooterWaitForRPM] Is shooter at target speed: " + isAtTargetSpeed);
    return isAtTargetSpeed;
    
  }
}
