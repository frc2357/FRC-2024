package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Robot;

public class ShooterPickupFromSource extends Command {
  public ShooterPickupFromSource() {
    addRequirements(Robot.shooter);
  }

  @Override
  public void initialize() {
    Robot.shooter.setRPMS(
        -SHOOTER.TOP_MOTOR_SOURCE_INTAKE_RPMS, -SHOOTER.BOTTOM_MOTOR_SOURCE_INTAKE_RPMS);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stop();
  }
}
