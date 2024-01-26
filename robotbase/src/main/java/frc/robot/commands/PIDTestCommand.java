package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CHOREO;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;

public class PIDTestCommand extends Command {
  int numOfLoops;
  Consumer<ChassisSpeeds> consumer;
  Supplier<Pose2d> supplier;

  double xSetpoint;
  double ySetpoint;
  double rotoSetpoint;
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
    double xOutput = CHOREO.Y_CONTROLLER.calculate(supplier.get().getY(), xSetpoint);
    double yOutput = CHOREO.Y_CONTROLLER.calculate(supplier.get().getY(), ySetpoint);
    consumer.accept(new ChassisSpeeds(xOutput, yOutput, 0));
    if (numOfLoops % 5 == 0) {
      System.out.println("Robot Pose X: " + Robot.drive.getPose().getX());
      System.out.println("Robot Pose Y: " + Robot.drive.getPose().getY());
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(supplier.get().getX() - 1) <= 0.01;
  }

  @Override
  public void end(boolean interupted) {
    Robot.drive.stopMotors();
    var pose = Robot.drive.getPose();
    var poseError = new Pose2d(xSetpoint, ySetpoint, new Rotation2d(rotoSetpoint)).minus(pose);
    System.out.println("Pose & Error ---");
    System.out.println("--X: " + pose.getX() + "\n| - Err: " + poseError.getX());
    System.out.println("--Y: " + pose.getY() + "\n| - Err: " + poseError.getY());
    System.out.println(
        "--Roto: "
            + pose.getRotation().getRadians()
            + "\n| - Err: "
            + poseError.getRotation().getRadians());
  }

  @Override
  public String toString() {
    return "PID Test Command";
  }
}
