package frc.robot.subsystems.intakeRoller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
    private TalonFX intakeMotor = new TalonFX(IntakeRollerConstants.INTAKE_MOTOR_ID);
    private TalonFX intakeFollower = new TalonFX(IntakeRollerConstants.INTAKE_MOTOR_FOLLOWER_ID);
    private VelocityTorqueCurrentFOC req = new VelocityTorqueCurrentFOC(0);

    StatusSignal<AngularVelocity> intakeVelocity = intakeMotor.getVelocity();
    StatusSignal<Current> intakeStator = intakeMotor.getStatorCurrent();
    StatusSignal<Current> intakeSupply = intakeMotor.getSupplyCurrent();

    StatusSignal<AngularVelocity> followerVelocity = intakeFollower.getVelocity();
    StatusSignal<Current> followerStator = intakeFollower.getStatorCurrent();
    StatusSignal<Current> followerSupply = intakeFollower.getSupplyCurrent();

    public IntakeRollerIOTalonFX() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 8;

        conf.CurrentLimits.StatorCurrentLimit = 45;
        conf.CurrentLimits.SupplyCurrentLimit = 45;

        intakeMotor.getConfigurator().apply(conf);

        intakeFollower.setControl(new Follower(IntakeRollerConstants.INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        BaseStatusSignal.refreshAll(intakeVelocity, intakeStator, intakeSupply, followerVelocity, followerStator, followerSupply);

        inputs.intakeSpinVelocity = intakeVelocity.getValue();
        inputs.intakeStatorCurrent = intakeStator.getValue();
        inputs.intakeSupplyCurrent = intakeSupply.getValue();

        inputs.followerSpinVelocity = followerVelocity.getValue();
        inputs.followerStatorCurrent = followerStator.getValue();
        inputs.followerSupplyCurrent = followerSupply.getValue();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        intakeMotor.setControl(req.withVelocity(velocity));
    }
}
