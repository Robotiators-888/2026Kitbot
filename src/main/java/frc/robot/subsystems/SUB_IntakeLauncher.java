package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;
public class SUB_IntakeLauncher extends SubsystemBase {
    private SparkMax intakeLauncherRoller;
    public Object stop;
    private double TargetLauncherRPM = 0;
    private SparkClosedLoopController LauncherController;

    
        @SuppressWarnings("removal")
        public SUB_IntakeLauncher() {
    
            intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);


            SparkMaxConfig launcherConfig = new SparkMaxConfig();

            launcherConfig.smartCurrentLimit(70);
            intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            double kP = 0.0004; // Aggressive P for rapid speed ramp
            double kI = 0.0;
            double kD = 0.0; 
            double kFF = 0.0024455696*1.15; // Based on NEO nominal RPM at 12V
            
            launcherConfig.closedLoop.pid(kP, kI, kD);
            launcherConfig.closedLoop.velocityFF(kFF);
            intakeLauncherRoller.configure(launcherConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
            
            LauncherController = intakeLauncherRoller.getClosedLoopController();
        
        }
    
    
        public void Intakeeject() {
            intakeLauncherRoller.set(-1*IntakeLauncher_Intake_Speed);
        }

        public void IntakeIntake() {
            intakeLauncherRoller.set(IntakeLauncher_Intake_Speed);
        }

        public void setLauncherRPM(double wheelRPM) {
            TargetLauncherRPM = wheelRPM;
            LauncherController.setReference(wheelRPM, ControlType.kVelocity);
    }
        public boolean UptoSpeed() {
            return Math.abs(TargetLauncherRPM - intakeLauncherRoller.getEncoder().getVelocity()) < 50;
        }



        
        public void stop() {
            intakeLauncherRoller.set(0);
        }

        public void SpinUpandLaunch() {
        intakeLauncherRoller.set(IntakeLauncher_Launching_Speed);
        }
    
        public void Intakeautointake() {    
            intakeLauncherRoller.set(IntakeLauncher_Auto_Launching_Speed);
            
        }

        public double LauncherRPM(){
            return intakeLauncherRoller.getEncoder().getVelocity(); 
        }

        public Command spinUpCommand() {
        return this.run(() -> SpinUpandLaunch());
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

        public void periodic() {
            SmartDashboard.putNumber("Launcher RPM", LauncherRPM());
            SmartDashboard.putNumber("Launcher DesiredRPM", TargetLauncherRPM);
            SmartDashboard.putBoolean("Launcher Up to Speed", UptoSpeed());
            SmartDashboard.putNumber("Launcher Voltage", intakeLauncherRoller.getBusVoltage());
            SmartDashboard.putNumber("Launcher Current", intakeLauncherRoller.getOutputCurrent());
            SmartDashboard.putNumber("Launcher Temp", intakeLauncherRoller.getMotorTemperature());
        
        }


}
