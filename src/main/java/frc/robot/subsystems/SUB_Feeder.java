// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkMax;



import edu.wpi.first.wpilibj.motorcontrol.Spark;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class SUB_Feeder extends SubsystemBase {
  private final Spark feederRoller;
  

  /** Creates a new CANBallSubsystem. */
  public SUB_Feeder() {
    // create brushed motors for each of the motors on the launcher mechanism
    feederRoller = new Spark(FEEDER_MOTOR_ID);
    
    feederRoller.setInverted(true);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.


    // // create the configuration for the feeder roller, set a current limit and apply
    // // the config to the controller
    // SparkConfig feederConfig = new SparkConfig();
    // feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    // feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // // create the configuration for the launcher roller, set a current limit, set
    // // the motor to inverted so that positive values are used for both intaking and
    // // launching, and apply the config to the controller
    // SparkConfig launcherConfig = new Sparkconfig();
    // launcherConfig.inverted(true);
    // launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    // intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the rollers to values for intaking
  public void eject() {
    feederRoller.set(Feeder_Eject_Speed);

  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void intake() {
    feederRoller
        .set(Feeder_Intake_Speed);

  }


  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);

  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher


  public void feed() {
    feederRoller.set(Feeder_Launching_Speed);
  }

  public void Autointakeeject() {
        feederRoller
        .set(Feeder_Auto_Intake_Speed);

  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem


  public Command feedCommand() {
    return this.run(() -> feed());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem


 public Command intakeCommand() {
  return this.run(() -> intake());
 }

 public Command ejectCommand() {
  return this.run(() -> eject());
 }

public Command AutointakeCommand() {
  return this.run(() -> Autointakeeject());
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
