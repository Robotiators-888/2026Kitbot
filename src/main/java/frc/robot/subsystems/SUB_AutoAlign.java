package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class SUB_AutoAlign {
    private final SUB_Drivetrain driveSubsystem = SUB_Drivetrain.getInstance();
    public Pose2d Bluehub;

    private SUB_AutoAlign() {
        Bluehub = new Pose2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32), null);
    }
}
