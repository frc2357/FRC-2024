package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EXTENSION_ARM;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;
import frc.robot.commands.state.SetRobotStateCommand;
import frc.robot.state.RobotState;

public class TrapPrepose extends SequentialCommandGroup {
  public TrapPrepose() {
    super(
        new NotePreload(),

        // Arm Prepose
        new ExtensionArmMoveToRotations(EXTENSION_ARM.TRAP_SCORE_ROTATIONS),
        new SetRobotStateCommand(RobotState.State.TRAP_PRE_POSE));
  }
}
