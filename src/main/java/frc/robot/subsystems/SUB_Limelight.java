package frc.robot.subsystems;



import static frc.robot.Constants.OperatorConstants.Auto_Align_Rotation_Speed;
import static frc.robot.Constants.LimelightConsants.*;
import static frc.robot.Constants.FuelConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SUB_Limelight extends SubsystemBase {    


    public SUB_Limelight() {
    LimelightHelpers.setCameraPose_RobotSpace("lime", Limelight_Forward_distance, Limelight_Side_distance, Limelight_Up_distance, Limelight_Roll_angle, Limelight_Pitch_angle, Limelight_Yaw_angle
        );
    LimelightHelpers.setPriorityTagID("lime", 9);
        }


    public void Autoalign() {

         //Sets the Limelight to only target the april tag with ID 9 which is on the Hub
        
        boolean Isseen = LimelightHelpers.getTV("lime"); //Says if an april tag is seen
        double degreesoff = LimelightHelpers.getTX("lime");
        LimelightHelpers.setLEDMode_ForceOn("lime");



        
        
        
        if (Isseen == true) {
            
            if (degreesoff < -5) { 
                while (degreesoff < -5) {
                    //Drivesubsystem.driveArcade(() -> Auto_Align_Rotation_Speed, () -> 0);
                    degreesoff = LimelightHelpers.getTX("lime");
                    LimelightHelpers.setLEDMode_ForceOn("lime");
                }
            }
            // If the robot is facing left it will turn right and then recheck if it is still facing too left
            if (degreesoff > 5) {
                while (degreesoff > 5) {
                    //Drivesubsystem.driveArcade(() -> (-1*Auto_Align_Rotation_Speed), () -> 0);
                    degreesoff = LimelightHelpers.getTX("lime");
                    LimelightHelpers.setLEDMode_ForceOn("lime");
                }
            }
        }
            // if the robot is facing right it will turn left and then check how it is currently facing
        

    }

    public Command AutoAlignCommand() {
        return this.run(() -> Autoalign());
        
}
}

