// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIDTest extends SubsystemBase {
  private TalonFX m_motor1, m_motor2;
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private final SysIdRoutine m_sysIdRoutine;
  /** Creates a new SysIDTest. */
  public SysIDTest(TalonFX motor1) {
    m_motor1 = motor1;

    m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
            Volts.of(0.4).per(Second),        // Use default ramp rate (1 V/s)
            Volts.of(2), // Reduce dynamic step voltage to 4 to prevent brownout
            null,        // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())
          ),
          new SysIdRoutine.Mechanism(
            (volts) -> {
                m_motor1.setControl(m_voltReq.withOutput(volts.in(Volts)));
                m_motor1.getVelocity();
            },
            null,
            this
          )
      );
  }

  public SysIDTest(TalonFX motor1, TalonFX motor2) {
    m_motor1 = motor1;
    m_motor2 = motor2;
   
    m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
            Volts.of(0.4).per(Second),        // Use default ramp rate (1 V/s)
            Volts.of(2), // Reduce dynamic step voltage to 4 to prevent brownout
            null,        // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())
          ),
          new SysIdRoutine.Mechanism(
            (volts) -> {
                m_motor1.setControl(m_voltReq.withOutput(volts.in(Volts)));
                m_motor1.getVelocity();
                m_motor2.setControl(m_voltReq.withOutput(volts.in(Volts)));
            },
            null,
            this
          )
      );
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
