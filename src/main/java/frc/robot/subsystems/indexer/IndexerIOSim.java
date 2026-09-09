package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {

    public double kGearRatio = 1;

    private VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);
    public final TalonFX motor = new TalonFX(IndexerConstants.INDEXER_MOTOR_ID);
    public final TalonFXSimState motorSim = motor.getSimState();

    StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    StatusSignal<Current> statorCurrent = motor.getStatorCurrent();
    StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent();

    DCMotorSim dcMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1),
                    0.01,
                    kGearRatio),
            DCMotor.getKrakenX60(1));

    public IndexerIOSim() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 4;

        conf.CurrentLimits.StatorCurrentLimit = 60;
        conf.CurrentLimits.SupplyCurrentLimit = 60;

        motor.getConfigurator().apply(conf);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {

        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double voltage = motorSim.getMotorVoltage();

        dcMotorSim.setInputVoltage(voltage);
        dcMotorSim.update(0.020);

        motorSim.setRawRotorPosition(dcMotorSim.getAngularPosition().times(kGearRatio));
        motorSim.setRotorVelocity(dcMotorSim.getAngularVelocity().times(kGearRatio));

        BaseStatusSignal.refreshAll(velocity, statorCurrent, supplyCurrent);
        inputs.indexerSpinVelocity = velocity.getValue();
        inputs.statorCurrent = statorCurrent.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(request.withVelocity(velocity));
    }
}