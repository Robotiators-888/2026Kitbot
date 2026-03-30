// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.FuelConstants.*;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Feeder;
import frc.robot.subsystems.SUB_IntakeLauncher;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SUB_Drivetrain driveSubsystem = SUB_Drivetrain.getInstance();
  private final SUB_Feeder ballSubsystem = new SUB_Feeder();
  private final SUB_IntakeLauncher intakeSubsystem = new SUB_IntakeLauncher();
  private final PowerDistribution PDP = new PowerDistribution();


  // The driver's controller
  public final CommandXboxController Driver1 = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  // private final CommandXboxController Driver2 = new CommandXboxController(
  //     OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
  }

  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    Driver1.leftBumper().whileTrue(
      new ParallelCommandGroup(
        (ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop())),
        (intakeSubsystem.runEnd(() -> intakeSubsystem.Intakeeject(), () -> intakeSubsystem.stop()))
    ));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    Driver1.rightTrigger()
      .whileTrue(new RunCommand(() -> intakeSubsystem.setLauncherRPM(2250), intakeSubsystem));
        // .whileTrue(intakeSubsystem.spinUpCommand()
        //     .finallyDo(() -> intakeSubsystem.stop()));
        
    Driver1.leftTrigger()
        .whileTrue(ballSubsystem.feedCommand()
            .finallyDo(() -> ballSubsystem.stop()));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    Driver1.rightBumper().whileTrue(
    new ParallelCommandGroup(
      (ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop())),
      (intakeSubsystem.runEnd(() -> intakeSubsystem.IntakeIntake(), () -> intakeSubsystem.stop())))
    );

    Driver1.a().onTrue(
      new InstantCommand(() ->driveSubsystem.navx.resetDisplacement())
    );




    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). The X-axis is also inverted so a positive value (stick to the right)
    // results in clockwise rotation (front of the robot turning right). Both axes
    // are also scaled down so the rotation is more easily controllable.
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveArcade(
            () -> -Driver1.getRightX() * DRIVE_SCALING,
            () -> -Driver1.getLeftY() * ROTATION_SCALING
            ));
    ballSubsystem.setDefaultCommand(
      new RunCommand(()->ballSubsystem.stop(),ballSubsystem)
    );
        intakeSubsystem.setDefaultCommand(
          new RunCommand(() -> intakeSubsystem.stop(),intakeSubsystem)
        );
  }
     

  public void periodic() {

    SmartDashboard.putNumber("Batery Voltage", PDP.getVoltage());
    SmartDashboard.putNumberArray("PDP channel Voltages", PDP.getAllCurrents());
    
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // An example command will be run in autonomous
    return new SequentialCommandGroup(
        // Drive backwards for 1 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
        Commands.parallel(
          driveSubsystem.driveArcade(() -> 0, ()-> .65).withTimeout(1),
          intakeSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
        ),
        
        // Stop driving. This line uses the regular driveArcade command factory so it
        // ends immediately after commanding the motors to stop
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(.01),

        Commands.repeatingSequence(
        intakeSubsystem.spinUpCommand().withTimeout(1),

        new ParallelCommandGroup(
          intakeSubsystem.spinUpCommand().withTimeout(.5),
          ballSubsystem.feedCommand().withTimeout(.5)

        )

        )
    );


  }
}
