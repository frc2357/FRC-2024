package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class ClimberRunThenZero extends SequentialCommandGroup {
  public ClimberRunThenZero() {
    super(
        new ParallelDeadlineGroup(
            new WaitCommand(Constants.CLIMBER.ZERO_SPEED_INITIAL_SECONDS),
            new ClimberSpeed(Constants.CLIMBER.ZERO_SPEED, Constants.CLIMBER.ZERO_SPEED)),
        new ClimberZero());
  }
}
