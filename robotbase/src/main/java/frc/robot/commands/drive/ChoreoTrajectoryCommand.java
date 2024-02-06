package frc.robot.commands.drive;

import com.choreo.lib.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;

public class ChoreoTrajectoryCommand extends SequentialCommandGroup {

  private String trajectoryFileName;
  private String pathName;
  private ChoreoTrajectory traj;
  private Pose2d finalTargetPose;

  /**
   * A utility command to run a Choreo path correctly.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public ChoreoTrajectoryCommand(String trajectoryFileName) {
    this(trajectoryFileName, trajectoryFileName);
  }

  /**
   * A utility command to run a Choreo path correctly.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param pathName The name of the path, is returned in the toString for the auto command chooser.
   */
  public ChoreoTrajectoryCommand(String trajectoryFileName, String pathName) {
    this.trajectoryFileName = trajectoryFileName;
    this.traj = Choreo.getTrajectory(trajectoryFileName);
    this.finalTargetPose = traj.getFinalPose();
    this.pathName = pathName;
    new Choreo();
    addCommands(
        new InstantCommand(() -> Robot.swerve.zeroAll()),
        new InstantCommand(() -> Robot.swerve.setPose(traj.getInitialPose())),
        Choreo.choreoSwerveCommand(
            Choreo.getTrajectory(trajectoryFileName),
            Robot.swerve.getPoseSupplier(),
            Choreo.choreoSwerveController(
                CHOREO.X_CONTROLLER, CHOREO.Y_CONTROLLER, CHOREO.ROTATION_CONTROLLER),
            Robot.swerve.getChassisSpeedsConsumer(),
            CHOREO.CHOREO_AUTO_MIRROR_PATHS,
            Robot.swerve),
        new InstantCommand(
            () -> {
              var pose = Robot.swerve.getPose();
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
    return pathName;
  }
}
