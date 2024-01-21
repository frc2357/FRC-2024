package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PIDTestCommand extends Command {
  int numOfLoops;
  Consumer<ChassisSpeeds> consumer;
  Supplier<Pose2d> supplier;

  public PIDTestCommand() {
    addRequirements(Robot.drive);
  }

  @Override
  public void initialize() {
    Robot.drive.zeroAll();
    consumer = Robot.drive.getChassisSpeedsConsumer();
    supplier = Robot.drive.getPoseSupplier();
    numOfLoops = 0;
  }

  @Override
  public void execute() {
    numOfLoops++;
    double output = CHOREO.CHOREO_X_CONTROLLER.calculate(supplier.get().getX(), 1);
    consumer.accept(new ChassisSpeeds(output, 0, 0));
    if (numOfLoops % 5 == 0) {
      System.out.println("Robot Pose X: " + Robot.drive.getPose().getX());
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(supplier.get().getX() - 1) <= 0.01;
  }

  @Override
  public void end(boolean interupted) {
    Robot.drive.stopMotors();
    System.out.println(
        "Robot Pose Reported On Finish | Interrupted = "
            + interupted
            + "\n "
            + Robot.drive.getPose().toString());
  }

  @Override
  public String toString() {
    return "PID Test Command";
  }
}
