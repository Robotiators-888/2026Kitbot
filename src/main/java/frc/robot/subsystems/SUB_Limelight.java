package frc.robot.subsystems;



import static frc.robot.Constants.OperatorConstants.Auto_Align_Rotation_Speed;
import static frc.robot.Constants.LimelightConsants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SUB_Limelight extends SubsystemBase {    

    private final SUB_Drivetrain Drivesubsystem = SUB_Drivetrain.getInstance();

    

    public void Autoalign() {

        LimelightHelpers.setCameraPose_RobotSpace("", Limelight_Forward_distance, Limelight_Side_distance, Limelight_Up_distance, Limelight_Roll_angle, Limelight_pitch_angle, Limelight_Yaw_angle);
        LimelightHelpers.SetFiducialIDFiltersOverride("", new int [9]);
         //Sets the Limelight to only target the april tag with ID 9 which is on the Hub
        double degreesoff = LimelightHelpers.getTX(""); //Determines how many degrees of the center of the camera is to the april tag
        boolean Isseen = LimelightHelpers.getTV(""); //Says if an april tag is seen

        if (Isseen == true) {
            if (degreesoff < 0) { 
                while (degreesoff < -2) {
                    Drivesubsystem.driveArcade(() -> Auto_Align_Rotation_Speed, () -> 0);
                    degreesoff = LimelightHelpers.getTX("");
                }
            }
            // If the robot is facing left it will turn right and then recheck if it is still facing too left
            if (degreesoff > 0) {
                while (degreesoff > 2) {
                    Drivesubsystem.driveArcade(() -> (-1*Auto_Align_Rotation_Speed), () -> 0);
                    degreesoff = LimelightHelpers.getTX("");
                }
            }
            // if the robot is facing right it will turn left and then check how it is currently facing
        }
        Drivesubsystem.driveArcade(() -> 0, () -> 0);
    }

    public Command AutoAlignCommand() {
        return this.run(() -> Autoalign());
}
}
