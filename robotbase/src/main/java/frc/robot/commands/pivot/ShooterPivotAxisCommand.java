package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ShooterPivotAxisCommand extends Command {
    private AxisInterface m_axis;

    public ShooterPivotAxisCommand(AxisInterface axis) {
        m_axis = axis;
        addRequirements(Robot.pivot);
    }

    @Override
    public void execute() {
        double axisSpeed = m_axis.getValue();
        Robot.pivot.setPivotAxisSpeed(axisSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.pivot.stop();
    }
    
}
