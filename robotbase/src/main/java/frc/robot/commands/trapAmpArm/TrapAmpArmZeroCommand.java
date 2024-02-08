package frc.robot.commands.trapAmpArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class TrapAmpArmZeroCommand extends Command{
    
    public TrapAmpArmZeroCommand(){
        addRequirements(Robot.trapAmpArm);
    }

    @Override
    public void initialize(){
        Robot.trapAmpArm.set(Constants.TRAP_AMP_ARM.ZERO_SPEED);
    }

    @Override
    public boolean isFinished(){
        return Robot.trapAmpArm.get
    }
}
