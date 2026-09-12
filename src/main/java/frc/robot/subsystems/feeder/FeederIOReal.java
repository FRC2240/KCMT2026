package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;

public class FeederIOReal implements FeederIO {
    private TalonFX feedMotor = new TalonFX(0);
    private StatusSignal<AngularVelocity> velocity = feedMotor.getVelocity();
    private VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);

    public FeederIOReal() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 2;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimit = 50;
        feedMotor.getConfigurator().apply(config);
    }
    
    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.velocity = velocity.getValue();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        feedMotor.setControl(velocityControl.withVelocity(velocity));
    }
}

