package frc.robot.util;

import frc.robot.Constants;
import frc.robot.Robot;

public class PipelineManager {
  public static int speakerPipeline() {
    return 4;
    // switch (Robot.state.getAlliance()) {
    // case Red:
    // return Constants.SHOOTER_LIMELIGHT.RED_SPEAKER_APRILTAG_PIPELINE_INDEX;
    // case Blue:
    // return Constants.SHOOTER_LIMELIGHT.BLUE_SPEAKER_APRILTAG_PIPELINE_INDEX;
    // default:
    // System.err.println("----- Alliance not selected -----");
    // return 0;
    // }
  }
}
