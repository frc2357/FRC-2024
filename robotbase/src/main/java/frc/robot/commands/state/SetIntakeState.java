package frc.robot.commands.state;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.state.RobotState.IntakeState;

public class SetIntakeState extends Command {
   private IntakeState m_state;

   public SetIntakeState(IntakeState state) {
      m_state = state;
   }

   @Override
   public void initialize() {
      Robot.state.setIntakeState(m_state);
   }

   @Override
   public boolean isFinished() {
      return true;
   }
}
