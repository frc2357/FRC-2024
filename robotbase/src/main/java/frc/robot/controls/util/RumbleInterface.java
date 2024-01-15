package frc.robot.controls.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public interface RumbleInterface {
    public void setRumble(RumbleType type, double intensity);
}
