package frc.robot.commands;

import com.choreo.lib.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;

public class ChoreoTrajectoryCommand extends SequentialCommandGroup {

  private String trajectoryFileName;
  private ChoreoTrajectory traj;
  private Pose2d finalTargetPose;

  /**
   * A utility command to run a Choreo path correctlly.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded
   */
  public ChoreoTrajectoryCommand(String trajectoryFileName) {
    this.trajectoryFileName = trajectoryFileName;
    this.traj = Choreo.getTrajectory(trajectoryFileName);
    this.finalTargetPose = traj.getFinalPose();
    new Choreo();
    addCommands(
        new InstantCommand(() -> Robot.drive.zeroAll()),
        new InstantCommand(() -> Robot.drive.setPose(traj.getInitialPose())),
        Choreo.choreoSwerveCommand(
            Choreo.getTrajectory(trajectoryFileName),
            Robot.drive.getPoseSupplier(),
            Choreo.choreoSwerveController(
                CHOREO.X_CONTROLLER, CHOREO.Y_CONTROLLER, CHOREO.ROTATION_CONTROLLER),
            Robot.drive.getChassisSpeedsConsumer(),
            CHOREO.CHOREO_AUTO_MIRROR_PATHS,
            Robot.drive),
        new InstantCommand(
            () -> {
              var pose = Robot.drive.getPose();
              var poseError = finalTargetPose.minus(pose);
              System.out.println("Pose & Error | PathName: " + trajectoryFileName);
              System.out.println("|X: " + pose.getX() + " | Err: " + poseError.getX());
              System.out.println("|Y: " + pose.getY() + " | Err: " + poseError.getY());
              System.out.println(
                  "|Roto: "
                      + pose.getRotation().getRadians()
                      + " | Err: "
                      + poseError.getRotation().getRadians());
            }));
  }

  @Override
  public String toString() {
    return trajectoryFileName;
  }
}
