package frc.robot.commands.trapAmpArm;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TrapAmpArmMoveToPositionCommand extends Command{
    private double m_position;
    public TrapAmpArmMoveToPositionCommand(double position){
        m_position = position;
        addRequirements(Robot.trapAmpArm);
    }

    @Override
    public void initialize(){
        Robot.trapAmpArm.setTargetRotations(m_position);
    }

    @Override
    public boolean isFinished(){
        return Robot.trapAmpArm.isAtTargetRotations();
    }

    @Override
    public void end(boolean interupted){
        Robot.trapAmpArm.stop();
    }
}
