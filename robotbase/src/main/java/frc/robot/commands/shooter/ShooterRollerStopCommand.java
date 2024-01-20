package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterRollerStopCommand extends Command {
    public ShooterRollerStopCommand() {
        addRequirements(Robot.shooter);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooter.stop();
    }
}
