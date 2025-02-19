package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FindingKS extends SubsystemBase{
    private TalonFX motor1;
    private TalonFX motor2;
    private Follower followerRequest;
    private double voltage = 0;
    private VoltageOut voltageRequest = new VoltageOut(0);
    private NeutralOut brakeRequest = new NeutralOut();
    private boolean usingPigeon;
    private boolean enabled = false;
    private static int instance = 0;
    private Pigeon2 pigeon;

    private int index = 0;
    private boolean initDone = false;
    private double[] voltages = new double[10];
    private double[] angles = new double[10];

    private double kS = -1.0;

    public FindingKS(int pigeonID, int motor1ID) {
        usingPigeon = pigeonID != -1;
        if(usingPigeon) pigeon = new Pigeon2(pigeonID);
        motor1 = new TalonFX(motor1ID);
        instance++;
    }
    public FindingKS(int pigeonID, int motor1ID, int motor2ID, boolean withOpposeMasterDirection) {
        usingPigeon = pigeonID != -1;
        if(usingPigeon) pigeon = new Pigeon2(pigeonID);
        motor1 = new TalonFX(motor1ID);
        motor2 = new TalonFX(motor2ID);
        followerRequest = new Follower(motor1ID, withOpposeMasterDirection);
        instance++;
    }

    public Command rampVoltage() {
        return Commands.sequence(
            Commands.runOnce(() -> enabled = true),
            Commands.run(() -> voltage += 0.2 / 50.0)
        );
    }
    public Command disable(double differenceToDetect) {
        if(initDone) {
            for (int i = 1; i < angles.length; i++) {
                if(angles[i - 1] + differenceToDetect < angles[i] ||
                    angles[i -1] - differenceToDetect < angles[i]
                ) kS = voltages[i];
            }
        }
        else {
            for (int i = 1; i < index + 1; i++) {
                if(angles[i - 1] + differenceToDetect < angles[i] ||
                    angles[i -1] - differenceToDetect < angles[i]
                ) kS = voltages[i];
            }
        }

        return Commands.runOnce(() -> {
            voltage = 0;
            enabled = false;
        });
    }

    public Command getKS() {
        return Commands.runOnce(() -> SmartDashboard.putNumber(instance + ": kS Value", kS));
    }

    public void periodic() {
        if(!enabled) {
            motor1.setControl(brakeRequest);
            if(motor2 != null) motor2.setControl(brakeRequest);
            return;
        }

        motor1.setControl(voltageRequest);
        if(motor2 != null) motor2.setControl(followerRequest);

        SmartDashboard.putNumber(instance + ": kS Voltage", voltage);
        SmartDashboard.putNumber(instance + ": kS Supply Current", motor1.getSupplyCurrent().getValueAsDouble() + (motor2 != null ? motor2.getSupplyCurrent().getValueAsDouble() : 0.0));
        if(usingPigeon) SmartDashboard.putNumber(instance + ": kS Angle", pigeon.getRoll().getValueAsDouble());
        else SmartDashboard.putNumber(instance + ": kS Encoder Angle", motor1.getPosition().getValueAsDouble());

        voltages[index] = voltage;
        angles[index] = usingPigeon ? pigeon.getRoll().getValueAsDouble() : motor1.getPosition().getValueAsDouble();
        index++;
        if(index >= voltages.length) {
            index = 0;
            initDone = true;
        }
    }
}
