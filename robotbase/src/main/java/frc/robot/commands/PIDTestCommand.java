package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.CHOREO;

public class PIDTestCommand extends Command {
  Consumer<ChassisSpeeds> consumer;
  @Override
  public void initialize() {
    consumer = Robot.drive.getChassisSpeedsConsumer();
    Robot.drive.tareEverything();
  }

  @Override
  public void execute() {
    consumer.accept(new ChassisSpeeds(CHOREO.CHOREO_X_CONTROLLER.calculate(Robot.drive.getPose().getX(), 1), 0, 0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interupted) {
    Robot.drive.stopMotors();
    System.out.println("Robot Pose Reported On Robot Stop---- Interrupted = "+ interupted +"\n " + Robot.drive.getPose().toString());
  }

  @Override
  public String toString() {
    return "Drive 1 Meter Test Command";
  }
}
