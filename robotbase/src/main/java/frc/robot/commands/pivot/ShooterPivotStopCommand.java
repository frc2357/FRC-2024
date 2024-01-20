package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterPivotStopCommand extends Command {
    public ShooterPivotStopCommand() {
        addRequirements(Robot.pivot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.pivot.stop();
    }
}
