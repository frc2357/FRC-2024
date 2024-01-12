package frc.robot.commands;
import com.choreo.lib.*;
import edu.wpi.first.wpilibj2.command.Command;

public class ChoreoTrajectoryCommand extends Command{

    private boolean stopMotorsWhenFinished;
    private String trajectoryFileName;
    public ChoreoTrajectoryCommand(String trajectoryFileName, boolean stopMotorsWhenFinished){
        this.stopMotorsWhenFinished = stopMotorsWhenFinished;
        this.trajectoryFileName = trajectoryFileName;
    }
    @Override
    public void initialize(){
        //TODO: finish this when swerve subsystem is done |new Choreo().choreoSwerveCommand(Choreo.getTrajectory(trajectoryFileName));
    }

    @Override
    public void end(boolean interrupted){
        if(stopMotorsWhenFinished){
            //TODO: add the stop swerve motors method call here, once its made.
        }
    }
}
