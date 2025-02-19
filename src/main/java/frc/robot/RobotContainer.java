// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FindingKS;
import frc.robot.util.Controller;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Controller driverController = new Controller(0);
  
  public final TalonFX motor = new TalonFX(8);
  public final TalonFX motor2 = new TalonFX(9);
  public final Follower follower = new Follower(8, false);
  public final Pigeon2 pigeon = new Pigeon2(2);
  public final VoltageOut voltageRequest = new VoltageOut(0);
  public final NeutralOut brakeRequest = new NeutralOut();
  public double voltage = .4;
  public FindingKS findingKS = new FindingKS(2, 8);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Wrist
    driverController.bumperRight()
      .whileTrue(findingKS.rampVoltage())
      .onFalse(findingKS.disable(voltage));
    driverController.bumperLeft()
      .onTrue(findingKS.getKS());

    driverController.triggerRight()
      .whileTrue(
        Commands.run(
            () -> {
              voltage += 0.2 / 50.0;
              motor.setControl(voltageRequest.withOutput(voltage));
            }
        )
      )
      .onFalse(
        Commands.sequence(
          Commands.runOnce(() -> motor.setControl(brakeRequest)),
          Commands.runOnce(() -> voltage = 0)
        )
      );

      // Elevator
      driverController.triggerLeft()
      .whileTrue(
        Commands.run(
          () -> {
            voltage += 0.2 / 50.0;
            motor.setControl(voltageRequest.withOutput(2));
            motor2.setControl(follower.withOpposeMasterDirection(false));
          }
        )
      )
      .onFalse(Commands.parallel(
        Commands.runOnce(() -> motor.setControl(brakeRequest)),
        Commands.runOnce(() -> motor2.setControl(brakeRequest)),
        Commands.runOnce(() -> voltage = 0)
      ));
      
      // Pivot
      driverController.bumperRight()
      .whileTrue(
        Commands.run(
          () -> {
            voltage += 0.2 / 50.0;
            motor.setControl(voltageRequest.withOutput(voltage));
            motor2.setControl(follower.withOpposeMasterDirection(true));
          }
      ))
      .onFalse(Commands.parallel(
        Commands.runOnce(() -> motor.setControl(brakeRequest)),
        Commands.runOnce(() -> motor2.setControl(brakeRequest)),
        Commands.runOnce(() -> voltage = 0)
      ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
