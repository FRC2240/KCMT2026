package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class IndexerIOTalonFX implements IndexerIO {
    private TalonFX motor = new TalonFX(IndexerConstants.INDEXER_MOTOR_ID);
    private VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);

    StatusSignal <AngularVelocity> velocity = motor.getVelocity();
    StatusSignal <Current> statorCurrent = motor.getStatorCurrent();
    StatusSignal <Current> supplyCurrent = motor.getSupplyCurrent();

    public IndexerIOTalonFX() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 4;

        conf.CurrentLimits.SupplyCurrentLimit = 100;
        conf.CurrentLimits.StatorCurrentLimit = 100;

        motor.getConfigurator().apply(conf);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
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

