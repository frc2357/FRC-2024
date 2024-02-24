package frc.robot.commands.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class SetCoastOnDisabled extends SequentialCommandGroup {
  public SetCoastOnDisabled() {
    new WaitCommand(5);
    new InstantCommand(
        () -> {
          Robot.swerve.configNeutralMode(NeutralModeValue.Coast);
        });
  }
}
