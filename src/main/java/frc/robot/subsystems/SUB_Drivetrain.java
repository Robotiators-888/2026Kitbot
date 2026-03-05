// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class SUB_Drivetrain extends SubsystemBase {
  private static SUB_Drivetrain INSTANCE = null;
  private final WPI_TalonSRX leftLeader;
  private final WPI_TalonSRX leftFollower;
  private final WPI_TalonSRX rightLeader;
  private final WPI_TalonSRX rightFollower;

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

    
    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);



  };

 @Override
  public void periodic() {
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }
}
