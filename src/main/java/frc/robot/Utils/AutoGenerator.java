package frc.robot.Utils;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SUB_Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;

public class AutoGenerator extends SubsystemBase {

    private final SUB_Drivetrain driveSubsystem = SUB_Drivetrain.getInstance();
    private static AutoGenerator INSTANCE = null;


    public AutoGenerator() {

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      throw new Error("robot config not loading");
      //return;
    }
    AutoBuilder.configure(
        driveSubsystem::getpose, // Robot pose supplier
        driveSubsystem::resetPose, // Method to reset odometry (will be called if your auto has starting pose)
        driveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveSubsystem.driveRobotRelative(speeds),
        new PPLTVController(0.02, Units.feetToMeters(10.91)), // PPLTVController is the built in path following controller for differential drive trains
            config,  // The robot configuration
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
            driveSubsystem // Reference to this subsystem to set requirements
        );

        registerAllCommands();
  }

  public void registerAllCommands() {}

  public static AutoGenerator getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new AutoGenerator();
    }

    return INSTANCE;
  }
    
}
