package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;
public class SUB_IntakeLauncher extends SubsystemBase {

    //Creates SparkMax object
    private SparkMax intakeLauncherRoller;

    
        @SuppressWarnings("removal")
        public SUB_IntakeLauncher() {


            //Defines SparkMax Object
            intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
            
            //Creates Config for launchermotor
            SparkMaxConfig launcherConfig = new SparkMaxConfig();

            //Sets currentlimit for motor and establishes persist mode which stops Sparkmax settings(Brushed/Brushless, ID, and Coast/Brushed) Not be lost
            launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
            intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    
    
        //Creates method for Intake motor to eject balls
        public void Intakeeject() {
            intakeLauncherRoller.set(INTAKER_EJECT_SPEED);
        }


        //Cretes method for Intake motor to intake balls
        public void IntakeIntake() {
            intakeLauncherRoller.set(INTAKER_INTAKING_SPEED);
        }


        //Creates method for Intake mtor to stop
        public void stop() {
            intakeLauncherRoller.set(0);
        }

        //Creates method to set speed for motor to launch balls at
        public void SpinUpandLaunch() {
        intakeLauncherRoller.set(INTAKER_LAUNCHING_SPEED);
        }
    
        //Creates command for running SpinUpandLaunch method
        public Command spinUpCommand() {
        return this.run(() -> SpinUpandLaunch());
        }

        //Creates command to eject balls
        public Command IntakeEjectCommand() {
            return this.run(() -> Intakeeject());
        }

        //Creates command to intake balls
        public Command IntakeIntakeCommand() {
            return this.run(() -> IntakeIntake());
        }


}
