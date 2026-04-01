// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  DifferentialDriveOdometry driveOdometry;

  private Rotation2d desiredangle;
  private Pose2d targetPose;
  public Pose2d tagPose;
  public double hubOffsetX;
  private Translation2d targetTranslation;
  private Translation2d Hubcentertranslation;

  
  public final AHRS navx;
  private Rotation2d pastYaw;
  public  DifferentialDriveKinematics m_kinematics;
  public final DifferentialDrivePoseEstimator m_poseEstimator;
  public DifferentialDriveWheelSpeeds wheelSpeeds;
  public DifferentialDriveWheelSpeeds past_wheelSpeeds;
  public double DistancetraveledLeft = 0;
  public double Distancetraveledright = 0;
  public Rotation2d angleoffset = new Rotation2d();
  public AprilTagFieldLayout at_field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  


  public final Field2d field;



  private final DifferentialDrive drive;
  public static SUB_Drivetrain getInstance(){
    if (INSTANCE ==null) {
      INSTANCE = new SUB_Drivetrain();
    }
    return INSTANCE;
  }
  private SUB_Drivetrain() {

    at_field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    tagPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
      ? at_field.getTagPose(10).orElse(new Pose3d()).toPose2d()
      : at_field.getTagPose(26).orElse(new Pose3d()).toPose2d();
    hubOffsetX = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Units.inchesToMeters(-23.5) : Units.inchesToMeters(23.5);
    Hubcentertranslation = new Translation2d(tagPose.getX() + hubOffsetX, tagPose.getY());
    targetPose = new Pose2d(Hubcentertranslation, new Rotation2d());
    targetTranslation = targetPose.getTranslation();
    // create brushed motors for drive

    leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
    leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
    rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
    rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);
    leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    pastYaw = navx.getRotation2d();

    driveOdometry = new DifferentialDriveOdometry(navx.getRotation2d(), leftLeader.getSelectedSensorPosition(), 
    rightLeader.getSelectedSensorPosition(),
    new Pose2d());

    leftLeader.configContinuousCurrentLimit(40);
    rightLeader.configContinuousCurrentLimit(40);
    leftFollower.configContinuousCurrentLimit(40);
    rightFollower.configContinuousCurrentLimit(40);

  

    LimelightHelpers.setCameraPose_RobotSpace("",
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
          new Rotation2d(0),
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

  public Rotation2d getdesiredangle() {
    return new Rotation2d(
      targetTranslation.getX() - m_poseEstimator.getEstimatedPosition().getX(),
      targetTranslation.getY() - m_poseEstimator.getEstimatedPosition().getY()
      );
  }

  public Boolean AutoAlign() {
    return (angleoffset.plus(navx.getRotation2d()).getDegrees() > getdesiredangle().getDegrees());
  }



  
  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }

  public void Rotate() {
    if (AutoAlign() == true) {
      driveArcade(() -> .2, () -> 0);
    } else {
      driveArcade(() -> -.2, () -> 0);
    }
  }


  StructPublisher<Pose2d> robotposepublisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose",Pose2d.struct).publish();

    StructPublisher<Pose2d> limelightposepublisher = NetworkTableInstance.getDefault()
    .getStructTopic("LLPose",Pose2d.struct).publish();

  

  public Pose2d getpose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    //zeroEncoders();
    DistancetraveledLeft = 0;
    Distancetraveledright = 0;
    m_poseEstimator.resetPosition(navx.getRotation2d(),DistancetraveledLeft, Distancetraveledright,
        pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(wheelSpeeds);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getpose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

    //convert to motor speed
    double leftVelocitySetpoint = wheelSpeeds.leftMetersPerSecond;
    double rightVelocitySetpoint = wheelSpeeds.rightMetersPerSecond;

    leftLeader.set(((leftVelocitySetpoint*12)/10.91)/leftLeader.getBusVoltage());
    rightLeader.set(-1*(((rightVelocitySetpoint*12)/10.91)/rightLeader.getBusVoltage()));

  }

 @Override
  public void periodic() {
    
    


    SmartDashboard.putNumber("Leftleader Current Draw", leftLeader.getSupplyCurrent());
    SmartDashboard.putNumber("RightLeader Current Draw", rightLeader.getSupplyCurrent());
    SmartDashboard.putNumber("RightLeader speed", rightLeader.get());
    SmartDashboard.putNumber("Leftleader speed", leftLeader.get());
    SmartDashboard.putNumber("Desired Angle", getdesiredangle().getDegrees());
    SmartDashboard.putNumber("Current Angle", angleoffset.plus(navx.getRotation2d()).getDegrees());
    SmartDashboard.putBoolean("test", AutoAlign());

    //wheelSpeeds = new DifferentialDriveWheelSpeeds(leftLeader.getSensorCollection().getQuadratureVelocity(), rightLeader.getSensorCollection().getQuadratureVelocity());
    wheelSpeeds = new DifferentialDriveWheelSpeeds(Units.feetToMeters((10.91 *(leftLeader.getBusVoltage()*leftLeader.get()/12))),Units.feetToMeters(10.91* (rightLeader.getBusVoltage()*-1*rightLeader.get()/12)));
    DistancetraveledLeft = DistancetraveledLeft + .02 * past_wheelSpeeds.leftMetersPerSecond;
    Distancetraveledright = Distancetraveledright + .02 * past_wheelSpeeds.rightMetersPerSecond;

   


    

    

    
        
   LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    if (limelightMeasurement.tagCount >= 1) {  // Only trust measurement if we see multiple tags
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds);
        
        navx.reset();
        angleoffset = limelightMeasurement.pose.getRotation();
        m_poseEstimator.addVisionMeasurement(limelightMeasurement.pose, Timer.getFPGATimestamp());
       
        
        

        
        
        
    }
  m_poseEstimator.update(
    angleoffset.plus(navx.getRotation2d()),
    DistancetraveledLeft, 
    Distancetraveledright);
      // In your periodic function:

  past_wheelSpeeds = wheelSpeeds;
  
  field.getRobotObject();
  field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  robotposepublisher.set(field.getRobotPose());

  limelightposepublisher.set(limelightMeasurement.pose);
  if (angleoffset.getDegrees()<navx.getRotation2d().getDegrees()) {

  }

  



   

  



}

    

}

