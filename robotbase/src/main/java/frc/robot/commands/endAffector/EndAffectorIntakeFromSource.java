package frc.robot.commands.endAffector;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;


public class EndAffectorIntakeFromSource extends Command {
    
    public EndAffectorIntakeFromSource(){
        addRequirements(Robot.endAffector);
    }

    @Override
    public void initialize(){
        Robot.endAffector.setSpeed(END_AFFECTOR.PICKUP_SPEED);
    }

    @Override
    public boolean isFinished(){
        return true; //Add the voltage dectection shenanigains
    }

    @Override
    public void end(boolean interrupted) {
        Robot.endAffector.setSpeed(0);
    }
}
