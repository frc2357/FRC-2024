package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ClimberZero extends Command {
  public boolean isLeftZeroed() {
    return Robot.climber.getLeftVelocity() <= Constants.CLIMBER.ZERO_SPEED_STOP_TOLERANCE;
  }

  public boolean isRightZeroed() {
    return Robot.climber.getRightVelocity() <= Constants.CLIMBER.ZERO_SPEED_STOP_TOLERANCE;
  }

  @Override
  public void initialize() {
    Robot.climber.setSpeed(Constants.CLIMBER.ZERO_SPEED, Constants.CLIMBER.ZERO_SPEED);
  }

  @Override
  public void execute() {
    double leftSpeed = isLeftZeroed() ? 0.0 : Constants.CLIMBER.ZERO_SPEED;
    double rightSpeed = isRightZeroed() ? 0.0 : Constants.CLIMBER.ZERO_SPEED;
    Robot.climber.setSpeed(leftSpeed, rightSpeed);
  }

  @Override
  public boolean isFinished() {
    return isLeftZeroed() && isRightZeroed();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climber.stop();

    if (!interrupted) {
      Robot.climber.setZero();
      System.out.println("[Climber] Zero set");
    } else {
      System.err.println("[Climber] Zero interrupted!");
    }
  }
}
