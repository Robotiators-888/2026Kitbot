package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SUB_AutoAlign {
    private final SUB_Drivetrain driveSubsystem = SUB_Drivetrain.getInstance();
    public Pose2d Bluehub;

    private SUB_AutoAlign() {
        Bluehub = new Pose2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32), null);
    }

        StructPublisher<Pose2d> hubposepuPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("HubPose",Pose2d.struct).publish();

    public void periodic() {
        hubposepuPublisher.set(Bluehub);
    }

}
