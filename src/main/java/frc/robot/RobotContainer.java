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

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.nio.channels.WritableByteChannel;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SysIDTest;

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
  
  public final DCMotorSim simulation = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 10.0), DCMotor.getKrakenX60Foc(1));
  public final TalonFX motor = new TalonFX(8);
  public final TalonFX motor2 = new TalonFX(9);
  public final Follower follower = new Follower(8, true);
  public final Pigeon2 pigeon = new Pigeon2(2);
  public final VoltageOut voltageRequest = new VoltageOut(0);
  public final NeutralOut brakeRequest = new NeutralOut();
  public final TalonFX wrist = new TalonFX(54);
  public final MotionMagicVoltage motionMagicController = new MotionMagicVoltage(0.0);
  private final SysIDTest systest = new SysIDTest(wrist);
  public double voltage = .4;
  public FindingKS findingKS = new FindingKS(2, 54);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    TalonFXConfigurator configurator = wrist.getConfigurator();
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configurator.refresh(configuration);
    configuration.Slot0.kP = 0.01;
    configuration.Slot0.kI = 0.0;
    configuration.Slot0.kD = 0.0;
    configuration.Slot0.kG = 0.0;
    configuration.Slot0.kS = 0.0;
    configuration.Slot0.kV = 0.0;
    configuration.Slot0.kA = 0.0;
    configurator.apply(configuration);
    // motionMagicController.FeedForward = ;
    StatusCode status = SignalLogger.setPath("/u/logs"); // TODO create folder
    if (!status.isOK()) {
      DriverStation.reportWarning("UHEFHUSDUFHOHS: " + status.getDescription(), false);
    }
    DriverStation.reportWarning("KSJHGkhfdfjgsdfjkgksdhg", false);

    configureBindings();
  }
  
  
  private void configureBindings() {
    driverController.buttonDown()
      .onTrue(Commands.runOnce(() -> {
        motor.setControl(voltageRequest);
        motor2.setControl(follower);
      }))
      .onFalse(Commands.runOnce(() -> {
        motor.setControl(brakeRequest);
        motor2.setControl(brakeRequest);
      }));
    driverController.buttonUp()
      .onTrue(Commands.runOnce(() -> {
        voltageRequest.Output = 0.0;
      }));
    driverController.triggerRight()
      .whileTrue(Commands.run(() -> {
        voltageRequest.Output += 0.1 / 50.0;
      }));
    driverController.triggerLeft()
      .whileTrue(Commands.run(() -> {
        voltageRequest.Output -= 0.1 / 50.0;
      }));
    driverController.bumperRight()
      .whileTrue(Commands.run(() -> {
        voltageRequest.Output += 0.01 / 50.0;
      }));
    driverController.bumperLeft()
      .whileTrue(Commands.run(() -> {
        voltageRequest.Output -= 0.01 / 50.0;
      }));
      ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
      tab.addDouble("Elevator Voltage", () -> voltageRequest.Output);
  }
    
    /**
     * up - rayquasi to
     * down - rayquasi fro
     * right - dynamike to
     * left - dynamike fro
     */
    private void configureSysIDBindings() {
      driverController.bumperLeft().onTrue(Commands.runOnce(SignalLogger::start));
      driverController.bumperRight().onTrue(Commands.runOnce(SignalLogger::stop));

      driverController.buttonUp().whileTrue(systest.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      driverController.buttonDown().whileTrue(systest.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      driverController.buttonRight().whileTrue(systest.sysIdDynamic(SysIdRoutine.Direction.kForward));
      driverController.buttonLeft().whileTrue(systest.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public void periodic() {
      SignalLogger.writeDouble("AngularVelocity", pigeon.getAngularVelocityXDevice().getValueAsDouble(), "Seconds");
      SignalLogger.writeDouble("Velocity", motor.getVelocity().getValueAsDouble(), "Seconds");
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
