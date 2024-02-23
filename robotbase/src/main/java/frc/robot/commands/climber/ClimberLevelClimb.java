package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CLIMBER;
import frc.robot.Robot;

public class ClimberLevelClimb extends Command {
  public ClimberLevelClimb() {
    addRequirements(Robot.climber);
  }

  @Override
  public void initialize() {
    CLIMBER.LEVEL_CLIMB_PID_CONTROLLER.setSetpoint(0);
  }

  @Override
  public void execute() {
    double roll = Robot.swerve.getRoll();
    double calculatedOffset = CLIMBER.LEVEL_CLIMB_PID_CONTROLLER.calculate(roll);

    double rightSpeed = CLIMBER.LEVEL_CLIMB_FEEDFORWARD;
    double leftSpeed = CLIMBER.LEVEL_CLIMB_FEEDFORWARD;

    if (roll >= CLIMBER.LEVEL_CLIMB_TOLERANCE / 2) {
      leftSpeed -= calculatedOffset;
    } else if (roll <= -CLIMBER.LEVEL_CLIMB_TOLERANCE / 2) {
      rightSpeed -= calculatedOffset;
    }

    // Robot.climber.set(rightSpeed, leftSpeed);
  }
}
