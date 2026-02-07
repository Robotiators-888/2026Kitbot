// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;

public class CANDriveSubsystem extends SubsystemBase {
  private final WPI_TalonSRX leftLeader;
  private final WPI_TalonSRX leftFollower;
  private final WPI_TalonSRX rightLeader;
  private final WPI_TalonSRX rightFollower;

  DifferentialDriveOdometry driveOdometry;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new WPI_TalonSRX(LEFT_LEADER_ID);
    leftFollower = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
    rightLeader = new WPI_TalonSRX(RIGHT_LEADER_ID);
    rightFollower = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);

    
    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    //leftLeader.setCANTimeout(250);
    // rightLeader.setCANTimeout(250);
    // leftFollower.setCANTimeout(250);
    // rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.


    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip

    // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    final Pose2d getPose() {
      return driveOdometry.getPoseMeters();
    }

  

  public DifferentialDrivePoseEstimator m_poseEstimator =
      new DifferentialDrivePoseEstimator(Constants.DriveConstants.KDriveKinematics,
      navx.getRotation2d(),
      leftLeaderEncoder.getPosition(),
      rightLeaderEncoder.getPosition(),
          new Pose2d(0, 0, new Rotation2d(0)));
    //TODO: cheak to see if done right /\

   public void setPosition(Pose2d position) {
    //driveOdometry.resetPosition(getGyroHeading(), this.rotationsToMeters(leftPrimaryEncoder.getPosition()), this.rotationsToMeters(rightSecondaryEncoder.getPosition()),
    //new Pose2d(0, 0, new Rotation2d()));
     //zeroEncoders();
     driveOdometry.resetPosition(navx.getRotation2d(), leftLeaderEncoder.getPosition(), rightLeaderEncoder.getPosition(), position);
   }
        
public void resetPose(Pose2d pose) {
    //zeroEncoders();
    driveOdometry.resetPosition(navx.getRotation2d(), leftLeaderEncoder.getPosition(), rightLeaderEncoder.getPosition(),
        pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.DriveConstants.KDriveKinematics.toChassisSpeeds(null);
  }//TODO: Replace getmoduelStates with the appropriate wheels speed  

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    //TODO:Find uses for TargetSpeeds, driveRobotRelative is needed.
  }


    // Configure AutoBuilder last
    AutoBuilderconfigure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new CommandXboxController(DRIVER_CONTROLLER_PORT), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
      
  };

  public void registerAllCommands() {}

  private Object driveRobotRelative(ChassisSpeeds speeds) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'driveRobotRelative'");
  }

  @Override
  public void periodic() {
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }
}
