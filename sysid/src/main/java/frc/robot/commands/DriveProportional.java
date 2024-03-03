package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DriveProportional extends Command {
    public DriveProportional() {
        super.addRequirements(Robot.drive);
    }

    @Override
    public void execute() {
        Robot.drive.driveProportionalWithStick(Robot.controller.getLeftY(), Robot.controller.getRightX());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}