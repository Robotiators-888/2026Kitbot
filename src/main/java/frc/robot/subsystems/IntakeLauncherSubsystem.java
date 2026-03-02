package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;
public class IntakeLauncherSubsystem extends SubsystemBase {
    private SparkMax intakeLauncherRoller;
    public Object stop;
    
        public IntakeLauncherSubsystem() {
    
            intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    
        }
    
    
        public void Intakeeject() {
            intakeLauncherRoller.set(-.6);
        }

        public void IntakeIntake() {
            intakeLauncherRoller.set(.6);
        }

        public void LauncherLauch() {
            intakeLauncherRoller.set(.7);
        }

        public void stop() {
            intakeLauncherRoller.set(0);
        }

        public void spinUp() {
        intakeLauncherRoller.set(.7);
        }
    
        public void Intakeautointake() {    
            intakeLauncherRoller.set(.1);
        }


        public Command spinUpCommand() {
        return this.run(() -> spinUp());
        }

        public Command IntakeautointakeCommand() {
            return this.run(() -> Intakeautointake());
        }

        public Command IntakeEjectCommand() {
            return this.run(() -> Intakeeject());
        }

        public Command IntakeIntakeCommand() {
            return this.run(() -> IntakeIntake());
        }

        public Command LauncherLaunchCommand() {
            return this.run(() -> LauncherLauch());
        }
}
