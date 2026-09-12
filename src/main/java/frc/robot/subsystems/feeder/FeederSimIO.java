package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Volts;

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

public class FeederSimIO implements FeederIO{

    public double kGearRatio = 1;


    private TalonFX feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID);
    TalonFXSimState talonFXSim = feederMotor.getSimState();

    private StatusSignal<AngularVelocity> velocity = feederMotor.getVelocity();
    private VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);

        DCMotorSim dcMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1),
                    0.01,
                    kGearRatio),
            DCMotor.getKrakenX60(1));

            public FeederSimIO() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 2;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimit = 50;
        feederMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = talonFXSim.getMotorVoltage();

        dcMotorSim.setInputVoltage(motorVoltage);
        dcMotorSim.update(0.020);

        talonFXSim.setRawRotorPosition(dcMotorSim.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(dcMotorSim.getAngularVelocity().times(kGearRatio));
        BaseStatusSignal.refreshAll(velocity);
        inputs.velocity = velocity.getValue();
        
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        feederMotor.setControl(velocityControl.withVelocity(velocity));
    }


}
