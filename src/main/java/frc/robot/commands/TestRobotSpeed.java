package frc.robot.commands;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class TestRobotSpeed extends Command{

    private final Drive drive;
    private final SpeedTestType type;

    public TestRobotSpeed(Drive drive, SpeedTestType type){
        this.drive = drive;
        this.type = type;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setModuleVoltages(new Voltage[]{type.equals(SpeedTestType.LINEAR) ? Constants.MAX_VOLTAGE : Constants.MAX_VOLTAGE.unaryMinus(), type.equals(SpeedTestType.LINEAR) ? Constants.MAX_VOLTAGE : Constants.MAX_VOLTAGE.unaryMinus(), Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE});
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    public enum SpeedTestType{
        LINEAR,
        ROTATION
    }
}
