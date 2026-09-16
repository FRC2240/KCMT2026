package frc.robot.subsystems.shooter;

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

public class ShooterIOSim implements ShooterIO {

    public double kGearRatio = 1.25;

    private VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);
    public final TalonFX leftUpperMotor = new TalonFX(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID);
    public final TalonFX leftLowerMotor = new TalonFX(ShooterConstants.LEFT_LOWER_FLYWHEEL_MOTOR_ID);
    public final TalonFX rightUpperMotor = new TalonFX(ShooterConstants.RIGHT_UPPER_FLYWHEEL_MOTOR_ID);
    public final TalonFX rightLowerMotor = new TalonFX(ShooterConstants.RIGHT_LOWER_FLYWHEEL_MOTOR_ID);

    public final TalonFXSimState leftUpperMotorSim = leftUpperMotor.getSimState();
    public final TalonFXSimState leftLowerMotorSim = leftLowerMotor.getSimState();
    public final TalonFXSimState rightUpperMotorSim = rightUpperMotor.getSimState();
    public final TalonFXSimState rightLowerMotorSim = rightLowerMotor.getSimState();
    
    public final TalonFXSimState motorSim = leftUpperMotor.getSimState();
    public final TalonFXSimState motorSim2 = leftLowerMotor.getSimState();
    public final TalonFXSimState motorSim3 = rightUpperMotor.getSimState();
    public final TalonFXSimState motorSim4 = rightLowerMotor.getSimState();

    StatusSignal<AngularVelocity> leftUpperVelocity = leftUpperMotor.getVelocity();
    StatusSignal<AngularVelocity> leftLowerVelocity = leftLowerMotor.getVelocity();
    StatusSignal<AngularVelocity> rightUpperVelocity = rightUpperMotor.getVelocity();
    StatusSignal<AngularVelocity> rightLowerVelocity = rightLowerMotor.getVelocity();

    StatusSignal<Current> leftUpperStatorCurrent = leftUpperMotor.getStatorCurrent();
    StatusSignal<Current> leftLowerStatorCurrent = leftLowerMotor.getStatorCurrent();
    StatusSignal<Current> rightUpperStatorCurrent = rightUpperMotor.getStatorCurrent();
    StatusSignal<Current> rightLowerStatorCurrent = rightLowerMotor.getStatorCurrent();

    StatusSignal<Current> leftUpperSupplyCurrent = leftUpperMotor.getSupplyCurrent();
    StatusSignal<Current> leftLowerSupplyCurrent = leftLowerMotor.getSupplyCurrent();
    StatusSignal<Current> rightUpperSupplyCurrent = rightUpperMotor.getSupplyCurrent();
    StatusSignal<Current> rightLowerSupplyCurrent = rightLowerMotor.getSupplyCurrent();

    DCMotorSim dcMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(4),
                    0.01,
                    kGearRatio),
            DCMotor.getKrakenX60(1));
    
    public ShooterIOSim() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 4;
        conf.Slot0.kI = 0.001;
        conf.Slot0.kD = 0.1;

        conf.CurrentLimits.SupplyCurrentLimit = 100;
        conf.CurrentLimits.StatorCurrentLimit = 140;

        leftLowerMotor.setControl(
            new Follower(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Aligned));
        
        rightUpperMotor.setControl(
            new Follower(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));
        
        rightLowerMotor.setControl(
            new Follower(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));

        leftUpperMotor.getConfigurator().apply(conf);

    }

    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftUpperVelocity, leftLowerVelocity, rightUpperVelocity, rightLowerVelocity,
            leftUpperStatorCurrent, leftLowerStatorCurrent, rightUpperStatorCurrent, rightLowerStatorCurrent,
            leftUpperSupplyCurrent, leftLowerSupplyCurrent, rightUpperSupplyCurrent, rightLowerSupplyCurrent
        );

        inputs.LeftUpperFlywheelVelocity = leftUpperVelocity.getValue();
        inputs.LeftLowerFlywheelVelocity = leftLowerVelocity.getValue();
        inputs.RightUpperFlywheelVelocity = rightUpperVelocity.getValue();
        inputs.RightLowerFlywheelVelocity = rightLowerVelocity.getValue();

        inputs.LeftUpperFlywheelStatorCurrent = leftUpperStatorCurrent.getValue();
        inputs.LeftLowerFlywheelStatorCurrent = leftLowerStatorCurrent.getValue();
        inputs.RightUpperFlywheelStatorCurrent = rightUpperStatorCurrent.getValue();
        inputs.RightLowerFlywheelStatorCurrent = rightLowerStatorCurrent.getValue();

        inputs.LeftUpperFlywheelSupplyCurrent = leftUpperSupplyCurrent.getValue();
        inputs.LeftLowerFlywheelSupplyCurrent = leftLowerSupplyCurrent.getValue();
        inputs.RightUpperFlywheelSupplyCurrent = rightUpperSupplyCurrent.getValue();
        inputs.RightLowerFlywheelSupplyCurrent = rightLowerSupplyCurrent.getValue();

        leftUpperMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        leftLowerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        rightUpperMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightLowerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double voltage = leftUpperMotorSim.getMotorVoltage();

        dcMotorSim.setInputVoltage(voltage);
        dcMotorSim.update(0.020);

        leftUpperMotorSim.setRawRotorPosition(dcMotorSim.getAngularPosition().times(kGearRatio));

    }
    @Override
    public void setVelocity(AngularVelocity velocity) {
        leftUpperMotor.setControl(request.withVelocity(velocity));
    }


}