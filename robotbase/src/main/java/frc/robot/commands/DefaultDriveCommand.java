package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DefaultDriveCommand extends Command {
    public DefaultDriveCommand() {
        addRequirements(Robot.drive);
    }

    @Override
    public void execute() {
        Robot.drive.drive(
                Robot.driverControls.getY(),
                Robot.driverControls.getX(),
                Robot.driverControls.getRotation());
    }

    @Override
    public void end(boolean interrupted) {
        Robot.drive.drive(0, 0, 0);
    }
}
