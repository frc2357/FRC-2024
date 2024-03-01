package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INTAKE;
import frc.robot.Robot;

//run after the intake sequence
//or after a shooter intake
//zero the note location
//move back and forth


//current thoughts: run upwards until it clears the beam, then continue upwards at a set time, then downwards until it hits the beam and repeat 4-5 times.

public class IntakeDecompressNote extends Command{
    
    Timer timer = new Timer();
    
    public IntakeDecompressNote(){
        addRequirements(Robot.intake);
    }

    @Override
    public void initialize(){
        Robot.intake.set(INTAKE.DECOMPRESS_SPEED_PERCENT_OUTPUT);

        timer.start();
    }

    @Override 
    public void execute(){
        if (!Robot.intake.isBeamBroken()) {
            
        }
    }

    @Override
    public boolean isFinished(){
        //return true;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stop();
    }

}
