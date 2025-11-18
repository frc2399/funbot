// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  public final CommandXboxController commandxboxcontroller = new CommandXboxController(0);
  public final CommandXboxController operatorXboxController = new CommandXboxController(1);

  private final double MAX_SPEED_METERS_PER_SECOND = 4.0;
  /*added array bc the code did not repeat the action when it was set to null. 
  /*it fixed it bc it told the code that there was nothing there.
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    setUpDefaultCommands();
    setUpDriverButtonBindings();
    setUpOperatorButtonBindings();
  }

  
 
  private void setUpDefaultCommands() {
    drivetrain.setDefaultCommand(Commands.run(
      () ->{
        drivetrain.setLeftSpeed(MetersPerSecond.of(commandxboxcontroller.getLeftY() * MAX_SPEED_METERS_PER_SECOND));
        drivetrain.setRightSpeed(MetersPerSecond.of(commandxboxcontroller.getRightY() * MAX_SPEED_METERS_PER_SECOND));
        //drivetrain.setLeftSpeed(MetersPerSecond.of(MAX_SPEED_METERS_PER_SECOND));
        //drivetrain.setRightSpeed(MetersPerSecond.of(MAX_SPEED_METERS_PER_SECOND));
        System.out.println("yay we did it");
  }
      
    , drivetrain)
      );
      
    
  }
 
  private void setUpDriverButtonBindings() {
    commandxboxcontroller.b().whileTrue(Commands.run(() -> System.out.println("yay we did it")));
  }

  

  
  private void setUpOperatorButtonBindings() {
    operatorXboxController.a().onTrue(Commands.none()).onFalse(Commands.none());
  }

}
