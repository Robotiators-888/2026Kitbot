package frc.robot.subsystems;



import static frc.robot.Constants.OperatorConstants.Auto_Align_Rotation_Speed;
import static frc.robot.Constants.LimelightConsants.*;
import static frc.robot.Constants.FuelConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SUB_Limelight extends SubsystemBase {    

 private final SUB_Drivetrain driveSubsystem = SUB_Drivetrain.getInstance();

    public SUB_Limelight() {
    LimelightHelpers.setCameraPose_RobotSpace("lime", Limelight_Forward_distance, Limelight_Side_distance, Limelight_Up_distance, Limelight_Roll_angle, Limelight_Pitch_angle, Limelight_Yaw_angle
        );



    
    }


    public void periodic() {
    
     driveSubsystem.m_poseEstimator.update(
        driveSubsystem.navx.getRotation2d(), driveSubsystem.wheelSpeeds.leftMetersPerSecond-driveSubsystem.past_wheelSpeeds.leftMetersPerSecond, 
            driveSubsystem.wheelSpeeds.rightMetersPerSecond-driveSubsystem.past_wheelSpeeds.rightMetersPerSecond);
            // In your periodic function:
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
        driveSubsystem.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        driveSubsystem.m_poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds
        );
        driveSubsystem.past_wheelSpeeds = driveSubsystem.wheelSpeeds;
    }



    }

}

