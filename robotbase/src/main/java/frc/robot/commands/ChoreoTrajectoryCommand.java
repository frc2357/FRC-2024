package frc.robot.commands;

import com.choreo.lib.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;

public class ChoreoTrajectoryCommand extends SequentialCommandGroup {

  private String trajectoryFileName;
  private ChoreoTrajectory traj;

  /**
   * A utility command to run a Choreo path correctlly.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded
   */
  public ChoreoTrajectoryCommand(String trajectoryFileName) {
    this.trajectoryFileName = trajectoryFileName;
    this.traj = Choreo.getTrajectory(trajectoryFileName);
    new Choreo();
    addCommands(
        new InstantCommand(() -> Robot.drive.tareEverything()),
        new InstantCommand(() -> Robot.drive.setPose(traj.getInitialPose())),
        Choreo.choreoSwerveCommand(
            Choreo.getTrajectory(trajectoryFileName),
            Robot.drive.getPoseSupplier(),
            Choreo.choreoSwerveController(
                CHOREO.CHOREO_X_CONTROLLER,
                CHOREO.CHOREO_Y_CONTROLLER,
                CHOREO.CHOREO_ROTATION_CONTROLLER),
            Robot.drive.getChassisSpeedsConsumer(),
            CHOREO.CHOREO_AUTO_MIRROR_PATHS,
            Robot.drive),
        new InstantCommand(() -> System.out.println("Robot Pose: " + Robot.drive.getPose())));
  }

  @Override
  public String toString() {
    return trajectoryFileName;
  }
}
