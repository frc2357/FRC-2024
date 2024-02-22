package frc.robot.commands.extensionArm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ExtensionArmZero extends Command {

  public ExtensionArmZero() {
    addRequirements(Robot.extensionArm);
  }

  @Override
  public void initialize() {
    Robot.extensionArm.set(Constants.EXTENSION_ARM.ZERO_SPEED);
  }

  @Override
  public boolean isFinished() {
    return Robot.extensionArm.getVelocity() <= Constants.EXTENSION_ARM.ZERO_SPEED_STOP_TOLERANCE;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.extensionArm.stop();

    if (!interrupted) {
      Robot.extensionArm.zeroArm();
    } else {
      DriverStation.reportError("Arm Extension Zero interrupted!", false);
    }
  }
}
