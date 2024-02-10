package frc.robot.commands.pickup;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterIntakeFromSource extends Command{
    public ShooterIntakeFromSource(){
        addRequirements(Robot.intake,Robot.shooter,Robot.pivot);
    }

    @Override
    public void initialize(){
        Robot.pivot.setPivotRotations(Constants.SHOOTER_PIVOT.INTAKE_FROM_SOURCE_ROTATIONS);
        Robot.intake.set(-Constants.INTAKE.TOP_MOTOR_FEED_SPEED_PERCENT_OUTPUT,-Constants.INTAKE.BOTTOM_MOTOR_FEED_SPEED_PERCENT_OUTPUT);
        Robot.shooter.setRPMS(-Constants.SHOOTER.TOP_MOTOR_PICKUP_SPEED_RPMS,-Constants.SHOOTER.BOTTOM_MOTOR_PICKUP_SPEED_RPMS);
   
    }

    @Override
    public boolean isFinished(){
        return Robot.intake.isBeamBroken();
    }

    @Override  
    public void end(boolean interrupted) {
        Robot.intake.stop();
        Robot.shooter.stop();
        Robot.pivot.stop();
    }

}