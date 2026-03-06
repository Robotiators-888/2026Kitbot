package frc.robot.subsystems;



import static frc.robot.Constants.OperatorConstants.Auto_Align_Rotation_Speed;
import static frc.robot.Constants.LimelightConsants.*;
import static frc.robot.Constants.FuelConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SUB_Limelight extends SubsystemBase {    

    private final SUB_Drivetrain Drivesubsystem = SUB_Drivetrain.getInstance();





    public void Autoalign() {

         //Sets the Limelight to only target the april tag with ID 9 which is on the Hub
        double degreesoff = LimelightHelpers.getTX(""); //Determines how many degrees of the center of the camera is to the april tag
        boolean Isseen = LimelightHelpers.getTV(""); //Says if an april tag is seen
        LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{9});

        LimelightHelpers.setLEDMode_ForceOff("");
        if (Isseen == true) {

            if (degreesoff < -15) { 
                while (degreesoff < -15) {
                    Drivesubsystem.driveArcade(() -> Auto_Align_Rotation_Speed, () -> 0);
                    degreesoff = LimelightHelpers.getTX("");
                    LimelightHelpers.setLEDMode_ForceOn("");
                }
            }
            // If the robot is facing left it will turn right and then recheck if it is still facing too left
            if (degreesoff > 15) {
                while (degreesoff > 15) {
                    Drivesubsystem.driveArcade(() -> (-1*Auto_Align_Rotation_Speed), () -> 0);
                    degreesoff = LimelightHelpers.getTX("");
                    LimelightHelpers.setLEDMode_ForceOn("");
                }
            }
        }
            // if the robot is facing right it will turn left and then check how it is currently facing
        

    }

    public Command AutoAlignCommand() {
        return this.run(() -> Autoalign());
}
}

