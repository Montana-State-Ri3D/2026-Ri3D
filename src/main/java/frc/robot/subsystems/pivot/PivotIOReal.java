package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOReal implements PivotIO{

    private SparkMax motor = new SparkMax(1, MotorType.kBrushless);
    private RelativeEncoder encoder;
    private SparkClosedLoopController closedLoop;

    public PivotIOReal(){
        closedLoop = motor.getClosedLoopController();
        encoder = motor.getAlternateEncoder();
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.angle = Rotation2d.fromRotations(encoder.getPosition());
        inputs.current = motor.getOutputCurrent();
        inputs.volts = motor.getBusVoltage();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        closedLoop.setReference(angle.getRotations(), ControlType.kPosition);
    }

    @Override
    public void configMotor(double p, double i, double d, double f) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pidf(p, i, d, f);
    }
}
