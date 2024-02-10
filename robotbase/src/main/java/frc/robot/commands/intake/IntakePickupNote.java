package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

<<<<<<<< HEAD:robotbase/src/main/java/frc/robot/commands/intake/IntakePickupNote.java
public class IntakePickupNote extends Command {

  public IntakePickupNote() {
========
public class IntakeNoteFromFloor extends Command {

  public IntakeNoteFromFloor() {
>>>>>>>> 68b8fdc (Added 3 new commands, along with a parallelRaceGroup to control them all):robotbase/src/main/java/frc/robot/commands/intake/IntakeNoteFromFloor.java
    addRequirements(Robot.intake);
  }

  @Override
  public void initialize() {
    Robot.intake.set(
        Constants.INTAKE.TOP_MOTOR_PICKUP_SPEED_PERCENT_OUTPUT,
        Constants.INTAKE.BOTTOM_MOTOR_PICKUP_SPEED_PERCENT_OUTPUT);
  }

  @Override
  public boolean isFinished() {
    return Robot.intake.isBeamBroken();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
