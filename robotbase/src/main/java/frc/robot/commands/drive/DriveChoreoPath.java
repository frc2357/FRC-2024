package frc.robot.commands.drive;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;

public class DriveChoreoPath extends SequentialCommandGroup {

  private String m_pathName;
  private ChoreoTrajectory m_traj;
  private Pose2d m_finalTargetPose;
  private ChoreoTrajectoryState m_startingState;

  /**
   * A utility command to run a Choreo path correctly.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public DriveChoreoPath(String trajectoryFileName) {
    this(trajectoryFileName, trajectoryFileName, true);
  }

  /**
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param pathName The name of the path, is returned in the toString for the auto command chooser.
   */
  public DriveChoreoPath(String trajectoryFileName, String pathName) {
    this(trajectoryFileName, pathName, true);
  }

  /**
   * A utility command to run a Choreo path correctly.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param pathName The name of the path, is returned in the toString for the auto command chooser.
   * @param setPoseToStartTrajectory Whether or not to set the robot pose to the paths starting
   *     trajectory.
   */
  public DriveChoreoPath(
      String trajectoryFileName, String pathName, boolean setPoseToStartTrajectory) {
    m_traj = Choreo.getTrajectory(trajectoryFileName);
    m_finalTargetPose = m_traj.getFinalPose();
    m_pathName = pathName;
    m_startingState = m_traj.getInitialState();
    new Choreo();

    if (setPoseToStartTrajectory) {
      addCommands(
          new InstantCommand(() -> Robot.swerve.zeroAll()),
          new InstantCommand(() -> Robot.swerve.setPose(m_traj.getInitialPose())));
    }

    addCommands(
        new InstantCommand(
            () ->
                Robot.swerve.driveFieldRelative(
                    m_startingState.velocityX,
                    m_startingState.velocityY,
                    m_startingState.angularVelocity)),
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
              var poseError = pose.minus(m_finalTargetPose);
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
    return m_pathName;
  }
}
