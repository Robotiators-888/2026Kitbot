// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import static frc.robot.Constants.DriveConstants.*;

import static frc.robot.Constants.LimelightConsants.*;


public class SUB_Drivetrain extends SubsystemBase {


  private static SUB_Drivetrain INSTANCE = null;
  private final WPI_TalonSRX leftLeader;
  private final WPI_TalonSRX leftFollower;
  private final WPI_TalonSRX rightLeader;
  private final WPI_TalonSRX rightFollower;
  public final AHRS navx;
  public  DifferentialDriveKinematics m_kinematics;
  public final DifferentialDrivePoseEstimator m_poseEstimator;
  public DifferentialDriveWheelSpeeds wheelSpeeds;
  public DifferentialDriveWheelSpeeds past_wheelSpeeds;


  public final Field2d field;



  private final DifferentialDrive drive;
  public static SUB_Drivetrain getInstance(){
    if (INSTANCE ==null) {
      INSTANCE = new SUB_Drivetrain();
    }
    return INSTANCE;
  }
  private SUB_Drivetrain() {
    // create brushed motors for drive

    leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
    leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
    rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
    rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

    navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    LimelightHelpers.setCameraPose_RobotSpace("lime",
          Limelight_Forward_distance, 
          Limelight_Side_distance, 
          Limelight_Up_distance, 
          Limelight_Roll_angle, 
          Limelight_Pitch_angle, 
          Limelight_Yaw_angle
        );


    field = new Field2d();
    field.setRobotPose(0, 0, navx.getRotation2d());
    

    past_wheelSpeeds = new DifferentialDriveWheelSpeeds(0,0);
    wheelSpeeds = new DifferentialDriveWheelSpeeds(0,0);


    

    m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23.0));


      m_poseEstimator =
      new DifferentialDrivePoseEstimator(
          m_kinematics,
          navx.getRotation2d(),
          0,
          0,
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    
    
    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);





  };




  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }

  StructPublisher<Pose2d> robotposepublisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose",Pose2d.struct).publish();


 @Override
  public void periodic() {
    wheelSpeeds = new DifferentialDriveWheelSpeeds(Units.feetToMeters(10.91)*leftLeader.get(), Units.feetToMeters(10.91)*rightLeader.get());
    

    m_poseEstimator.update(
        navx.getRotation2d(), .02 * wheelSpeeds.leftMetersPerSecond, 
            .02 * past_wheelSpeeds.rightMetersPerSecond);
            // In your periodic function:
 
        past_wheelSpeeds = wheelSpeeds;

    
   LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("lime");
    if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds
        );
    //field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    field.setRobotPose(LimelightHelpers.getBotPose2d_wpiBlue(""));



    
    robotposepublisher.set(field.getRobotPose());

    
    
    // Convert to chassis speeds.

  }
}
}
