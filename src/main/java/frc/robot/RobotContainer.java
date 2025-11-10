// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
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
  /*added array bc the code did not repeat the action when it was set to null. 
  /*it fixed it bc it told the code that there was nothing there.
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    setUpDefaultCommands();
    setUpDriverButtonBindings();
    setUpOperatorButtonBindings();
  }
 
  private void setUpDefaultCommands() {
    
  }
 
  private void setUpDriverButtonBindings() {
    commandxboxcontroller.a().whileTrue(Commands.run(() -> System.out.println("yay we did it")));
  }

  private void setUpOperatorButtonBindings() {

  }

}
