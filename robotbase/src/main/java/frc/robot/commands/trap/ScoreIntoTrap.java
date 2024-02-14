package frc.robot.commands.trap;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.END_AFFECTOR;
import frc.robot.Constants.TRAP_AMP_ARM;
import frc.robot.commands.endAffector.EndAffectorSetSpeed;
import frc.robot.commands.endAffector.EndAffectorStop;
import frc.robot.commands.extensionArm.ExtensionArmMoveToRotations;

public class ScoreIntoTrap extends SequentialCommandGroup{
    public ScoreIntoTrap(){
                addCommands(
            new ExtensionArmMoveToRotations(TRAP_AMP_ARM.TRAP_SCORE_ROTATIONS),
            new EndAffectorSetSpeed(END_AFFECTOR.SCORE_SPEED_TRAP),
            new WaitCommand(END_AFFECTOR.TIME_TO_SCORE_TRAP),
            new EndAffectorStop(),
            new ExtensionArmMoveToRotations(TRAP_AMP_ARM.HOME_ROTATIONS)
        );
    }
}
