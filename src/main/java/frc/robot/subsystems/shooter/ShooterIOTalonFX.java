package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterIOTalonFX implements ShooterIO {
    private TalonFX leftFlywheelMotor = new TalonFX(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID);
    private TalonFX leftFlywheelFollower  = new TalonFX(ShooterConstants.LEFT_LOWER_FLYWHEEL_MOTOR_ID);
    
    private TalonFX rightFlywheelFollowerLower = new TalonFX(ShooterConstants.RIGHT_UPPER_FLYWHEEL_MOTOR_ID);
    private TalonFX rightFlywheelFollowerUpper  = new TalonFX(ShooterConstants.RIGHT_LOWER_FLYWHEEL_MOTOR_ID);

    private VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);

    StatusSignal <AngularVelocity> leftUpperVelocity = leftFlywheelMotor.getVelocity();
    StatusSignal <AngularVelocity> leftLowerVelocity = leftFlywheelFollower.getVelocity();
    StatusSignal <AngularVelocity> rightUpperVelocity = rightFlywheelFollowerLower.getVelocity();
    StatusSignal <AngularVelocity> rightLowerVelocity = rightFlywheelFollowerUpper.getVelocity();

    StatusSignal <Current> leftUpperStatorCurrent = leftFlywheelMotor.getStatorCurrent();
    StatusSignal <Current> leftLowerStatorCurrent = leftFlywheelFollower.getStatorCurrent();
    StatusSignal <Current> rightUpperStatorCurrent = rightFlywheelFollowerLower.getStatorCurrent();
    StatusSignal <Current> rightLowerStatorCurrent = rightFlywheelFollowerUpper.getStatorCurrent();

    StatusSignal <Current> leftUpperSupplyCurrent = leftFlywheelMotor.getSupplyCurrent();
    StatusSignal <Current> leftLowerSupplyCurrent = leftFlywheelFollower.getSupplyCurrent();
    StatusSignal <Current> rightUpperSupplyCurrent = rightFlywheelFollowerLower.getSupplyCurrent();
    StatusSignal <Current> rightLowerSupplyCurrent = rightFlywheelFollowerUpper.getSupplyCurrent();

    public void Shooter() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 4;
        conf.Slot0.kI = 0.001;
        conf.Slot0.kD = 0.1;

        conf.CurrentLimits.SupplyCurrentLimit = 100;
        conf.CurrentLimits.StatorCurrentLimit = 140;

        leftFlywheelFollower.setControl(
            new Follower(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Aligned));
        
        rightFlywheelFollowerUpper.setControl(
            new Follower(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));
        
        rightFlywheelFollowerLower.setControl(
            new Follower(ShooterConstants.LEFT_UPPER_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));

        leftFlywheelMotor.getConfigurator().apply(conf);
        leftFlywheelFollower.getConfigurator().apply(conf);
        rightFlywheelFollowerUpper.getConfigurator().apply(conf);
        rightFlywheelFollowerLower.getConfigurator().apply(conf);
    }

    @Override
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
    }
    @Override
    public void setVelocity(AngularVelocity velocity) {
        leftFlywheelMotor.setControl(request.withVelocity(velocity));
    }
}
