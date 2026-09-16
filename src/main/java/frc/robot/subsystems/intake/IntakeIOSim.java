package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

    public double kGearRatio = 1;

    private VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);
    private final TalonFX intakeMotor = new TalonFX(IntakeIOConstants.INTAKE_MOTOR_ID);
    private final TalonFX intakeFollower = new TalonFX(IntakeIOConstants.INTAKE_MOTOR_FOLLOWER_ID);

    public final TalonFXSimState motorSim = intakeMotor.getSimState();
    private final TalonFXSimState intakeFollowerSim = intakeFollower.getSimState();

    StatusSignal<AngularVelocity> intakeVelocity = intakeMotor.getVelocity();
    StatusSignal<Current> statorCurrent = intakeMotor.getStatorCurrent();
    StatusSignal<Current> supplyCurrent = intakeMotor.getSupplyCurrent();

    StatusSignal<AngularVelocity> followerVelocity = intakeFollower.getVelocity();
    StatusSignal<Current> followerStatorCurrent = intakeFollower.getStatorCurrent();
    StatusSignal<Current> followerSupplyCurrent = intakeFollower.getSupplyCurrent();

    DCMotorSim dcMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(2),
                    0.01,
                    kGearRatio),
            DCMotor.getKrakenX60(2));

    public IntakeIOSim() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 4;

        conf.CurrentLimits.StatorCurrentLimit = 60;
        conf.CurrentLimits.SupplyCurrentLimit = 60;

        intakeMotor.getConfigurator().apply(conf);

        intakeFollower.setControl(new Follower(IntakeIOConstants.INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);
        intakeFollowerSim.setSupplyVoltage(batteryVoltage);

        double voltage = motorSim.getMotorVoltage();

        dcMotorSim.setInputVoltage(voltage);
        dcMotorSim.update(0.020);

        motorSim.setRawRotorPosition(dcMotorSim.getAngularPosition().times(kGearRatio));
        motorSim.setRotorVelocity(dcMotorSim.getAngularVelocity().times(kGearRatio));

        intakeFollowerSim.setRawRotorPosition(dcMotorSim.getAngularPosition().times(kGearRatio));
        intakeFollowerSim.setRotorVelocity(dcMotorSim.getAngularVelocity().times(kGearRatio));

        BaseStatusSignal.refreshAll(intakeVelocity, statorCurrent, supplyCurrent, followerVelocity, followerStatorCurrent, followerSupplyCurrent);
        inputs.intakeSpinVelocity = intakeVelocity.getValue();
        inputs.intakeStatorCurrent = statorCurrent.getValue();
        inputs.intakeSupplyCurrent = supplyCurrent.getValue();

        inputs.followerSpinVelocity = followerVelocity.getValue();
        inputs.followerStatorCurrent = followerStatorCurrent.getValue();
        inputs.followerSupplyCurrent = followerSupplyCurrent.getValue();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        intakeMotor.setControl(request.withVelocity(velocity));
    }
}