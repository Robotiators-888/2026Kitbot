// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;





import edu.wpi.first.wpilibj.motorcontrol.Spark;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class SUB_Feeder extends SubsystemBase {
  //Creates feederRoller Motor
  private final Spark feederRoller;
  

  /** Creates a new CANFuelSubsystem. */
  public SUB_Feeder() {
    // create brushed motors for each of the motors on the launcher mechanism
    feederRoller = new Spark(FEEDER_MOTOR_ID);
    
    //Makes feederRoller Inverted which makes the motor spin opposite of how it was set
    feederRoller.setInverted(true);

   
  }

  // A method to set the rollers to values for intaking
  public void eject() {
    feederRoller.set(FEEDER_EJECT_SPEED);

  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void intake() {
    feederRoller
        .set(FEEDER_INTAKE_SPEED);

  }


  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);

  }



  //A method to feed balls into the Launcher
  public void feed() {
    feederRoller.set(FEEDER_LAUNCHING_SPEED);
  }


  //A seperate method to intake during auto
  public void Autofeederintake() {
        feederRoller
        .set(FEEDER_LAUNCHING_SPEED);

  }



  //A command to run the feed method
  public Command feedCommand() {
    return this.run(() -> feed());
  }



//A command to run the intake method
 public Command intakeCommand() {
  return this.run(() -> intake());
 }

 //A command to run the eject method
 public Command ejectCommand() {
  return this.run(() -> eject());
 }


 //A command to run the Auto intake method
public Command AutointakeCommand() {
  return this.run(() -> Autofeederintake());
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
