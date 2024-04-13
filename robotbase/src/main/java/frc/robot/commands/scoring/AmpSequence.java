package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDs.LEDsSetIdle;
import frc.robot.commands.endAffector.EndAffectorStop;
import frc.robot.commands.extensionArm.ExtensionArmReturnToZero;
import frc.robot.commands.util.PressToContinue;

public class AmpSequence extends SequentialCommandGroup {
  public AmpSequence(Trigger continueButton) {
    super(
        new AmpPrepose(),
        new PressToContinue(continueButton),
        new AmpScore()
            .handleInterrupt(
                () -> {
                  new ParallelCommandGroup(new ExtensionArmReturnToZero(), new EndAffectorStop())
                      .schedule();
                }),
        new LEDsSetIdle());
  }
}
