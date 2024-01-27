package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PIDTestCommand extends Command {
  int numOfLoops;
  Consumer<ChassisSpeeds> consumer;
  Supplier<Pose2d> supplier;

  double xSetpoint;
  double ySetpoint;
  double rotoSetpoint;
  PIDController controller;

  public PIDTestCommand() {
    addRequirements(Robot.drive);
  }

  @Override
  public void initialize() {
    Robot.drive.zeroAll();
    controller = new PIDController(0.0001, 0, 0);
    consumer = Robot.drive.getChassisSpeedsConsumer();
    supplier = Robot.drive.getPoseSupplier();
    numOfLoops = 0;
    System.out.println("Running PID Test Command ---");
  }

  @Override
  public void execute() {
    numOfLoops++;
    // consumer.accept(new ChassisSpeeds(controller.calculate(Robot.drive.getPose().getX()), 0, 0));
    consumer.accept(new ChassisSpeeds(0, 0, 0));
    /*if (numOfLoops % 5 == 0) {
      System.out.println("Robot Pose X: " + Robot.drive.getPose().getX());
      System.out.println("Robot Pose Y: " + Robot.drive.getPose().getY());
    }*/
  }

  @Override
  public boolean isFinished() {
    return Math.abs(supplier.get().getX() - 1) <= 0.01;
  }

  @Override
  public void end(boolean interupted) {
    Robot.drive.stopMotors();
    System.out.println("PID Test Command End ---");
    /*var pose = Robot.drive.getPose();
    var poseError = new Pose2d(xSetpoint, ySetpoint, new Rotation2d(rotoSetpoint)).minus(pose);
    System.out.println("Pose & Error ---");
    System.out.println("--X: " + pose.getX() + "\n| - Err: " + poseError.getX());
    System.out.println("--Y: " + pose.getY() + "\n| - Err: " + poseError.getY());
    System.out.println(
        "--Roto: "
            + pose.getRotation().getRadians()
            + "\n| - Err: "
            + poseError.getRotation().getRadians());*/
  }

  @Override
  public String toString() {
    return "PID Test Command";
  }
}
