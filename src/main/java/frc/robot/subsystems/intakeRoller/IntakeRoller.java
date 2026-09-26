package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class IntakeRoller extends SubsystemBase {
    private IntakeRollerIO io;
    private IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    private final Drivetrain drivetrain;

    public IntakeRoller(IntakeRollerIO io, Drivetrain drivetrain) {
      this.io = io;
      this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake Roller", inputs);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    public Command setIntakeVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    public Command setIntakeVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runOnce(() -> setVelocity(velocity.get()));
    }

    public Command enableIntakeCommand() {
        return Commands.repeatingSequence(
                run(() -> {
                    double MAX_ROBOT_SPEED = 4.5;
                    double MIN_ROLLER_SPEED = 50;
                    double MAX_ROLLER_SPEED = 90;
                    ChassisSpeeds speeds = drivetrain.getChassisSpeeds();

                    double speed = Math
                            .sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
                    speed = Math.min(speed, MAX_ROBOT_SPEED); // Caps to 4.5 m/s
                    double fraction = speed / MAX_ROBOT_SPEED;
                    double desiredRollerSpeed = MIN_ROLLER_SPEED + (MAX_ROLLER_SPEED - MIN_ROLLER_SPEED) * fraction;
                    SmartDashboard.putNumber("Intake/Desired Roller Speed", -desiredRollerSpeed);

                    io.setVelocity((RotationsPerSecond.of(-desiredRollerSpeed)));
                }));
    }
 
    public Command disableIntakeCommand() {
        return setIntakeVelocityCommand(RotationsPerSecond.of(0));
    }

    public Command reverseIntakeCommand() {
        return setIntakeVelocityCommand(IntakeRollerConstants.INTAKE_VELOCITY.unaryMinus());
    }

    public Command slowReverseIntakeCommand() {
        return setIntakeVelocityCommand(RotationsPerSecond.of(1));
    }

    public Command ReverseIntakeCommand() {
        return runOnce(() -> setIntakeVelocityCommand(RotationsPerSecond.of(-1)));
    }
}

