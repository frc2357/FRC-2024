package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoPivotSetAngle;
import frc.robot.commands.auto.AutoPivotStop;
import frc.robot.commands.auto.AutoShooterSetRPMAndFinish;
import frc.robot.commands.auto.AutoShooterStopRPM;
import frc.robot.commands.drive.DriveChoreoPath;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.shooter.ShooterSetRPM;
import frc.robot.commands.shooter.ShooterWaitForRPM;

public class RightClose1Speaker extends SequentialCommandGroup{
    public RightClose1Speaker(){
        super(
            new AutoPivotSetAngle(60),
            new AutoShooterSetRPMAndFinish(3000),
            new ShooterWaitForRPM().withTimeout(0.5),
            new IntakeRun(0.75, false).withTimeout(1),

            // Run path
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new AutoPivotSetAngle(35),
                    new DriveChoreoPath("RightClose1Speaker.1")
                )
            )
        );
    }
}
