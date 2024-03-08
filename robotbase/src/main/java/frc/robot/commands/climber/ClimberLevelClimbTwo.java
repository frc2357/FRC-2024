package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CLIMBER;
import frc.robot.Robot;

public class ClimberLevelClimbTwo extends Command {
  private double m_startingRollOffset;

  public ClimberLevelClimbTwo() {
    addRequirements(Robot.climber);
  }

  @Override
  public void initialize() {
    m_startingRollOffset = Robot.swerve.getRoll();
    CLIMBER.LEVEL_CLIMB_PID_CONTROLLER_TWO.setSetpoint(m_startingRollOffset);
    CLIMBER.LEVEL_CLIMB_PID_CONTROLLER_TWO.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    double axisSpeed = 0.0;

    if (Robot.codriverControls.getRightTriggerAxis() > 0) {
      axisSpeed = Robot.codriverControls.getRightTriggerAxis();
    }
    if (Robot.codriverControls.getLeftTriggerAxis() > 0) {
      axisSpeed = Robot.codriverControls.getLeftTriggerAxis();
    }

    double roll = Robot.swerve.getRoll();
    double calculatedOffset = Math.abs(CLIMBER.LEVEL_CLIMB_PID_CONTROLLER_TWO.calculate(roll));

    double rightSpeed = CLIMBER.LEVEL_CLIMB_MAX * axisSpeed;
    double leftSpeed = CLIMBER.LEVEL_CLIMB_MAX * axisSpeed;

    leftSpeed += calculatedOffset;

    Robot.climber.setSpeed(leftSpeed, rightSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climber.stop();
  }
}
