package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Utility;

public class ForceGyroZero extends Command {

  public ForceGyroZero() {}

  @Override
  public void execute() {
    Robot.swerve.zeroGyro(false);
    System.out.println("TYRING TO ZERO GYRO");
  }

  @Override
  public boolean isFinished() {
    return Utility.isWithinTolerance(Robot.swerve.getYaw(), 0, 0.00001);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
