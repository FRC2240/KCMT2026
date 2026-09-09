package frc.robot.subsystems.drivetrain.Feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;

public class FeederIOReal implements FeederIO {
    TalonFX motor = new TalonFX(0);
    StatusSignal<AngularVelocity> velocity = motor.getVelocity();
    VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);


    public void updateInputs(FeederIOInputs inputs) {
        inputs.velocity = velocity.getValue();
    }

    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityControl.withVelocity(velocity));
    }
}

