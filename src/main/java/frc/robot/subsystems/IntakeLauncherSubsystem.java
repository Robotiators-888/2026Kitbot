package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;
public class IntakeLauncherSubsystem extends SubsystemBase {
    private Spark intakeLauncherRoller;
    public Object stop;
    
        public IntakeLauncherSubsystem() {
    
            intakeLauncherRoller = new Spark(INTAKE_LAUNCHER_MOTOR_ID);
    
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
