package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CLIMBER;
import frc.robot.Robot;

public class ClimberLevelClimb extends Command {
  private double m_startingRollOffset;

  public ClimberLevelClimb() {
    addRequirements(Robot.climber);
  }

  @Override
  public void initialize() {
    m_startingRollOffset = Robot.swerve.getRoll();
    CLIMBER.LEVEL_CLIMB_PID_CONTROLLER.setSetpoint(m_startingRollOffset);
    CLIMBER.LEVEL_CLIMB_PID_CONTROLLER.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    double roll = Robot.swerve.getRoll();
    double calculatedOffset = Math.abs(CLIMBER.LEVEL_CLIMB_PID_CONTROLLER.calculate(roll));

    double rightSpeed = CLIMBER.LEVEL_CLIMB_FEEDFORWARD;
    double leftSpeed = CLIMBER.LEVEL_CLIMB_FEEDFORWARD;

    // Notes for Tyson
    // negative roll: slow down the left side
    // positive roll: slow down the right side
    // left and right are backwards relative to the stage because intake is facing
    // away from stage
    if (roll >= m_startingRollOffset + CLIMBER.LEVEL_CLIMB_TOLERANCE / 2) {
      rightSpeed -= calculatedOffset;
    } else if (roll <= m_startingRollOffset - CLIMBER.LEVEL_CLIMB_TOLERANCE / 2) {
      leftSpeed -= calculatedOffset;
    }
    // System.out.println("Roll: " + roll + ", Offset: " + calculatedOffset + ",
    // LeftSpeed: " + leftSpeed
    // + ", RightSpeed: " + rightSpeed + ", RollOffset: " + m_startingRollOffset);
    System.out.println(Robot.swerve.getPitch());

    Robot.climber.set(rightSpeed, leftSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climber.stop();
  }
}
