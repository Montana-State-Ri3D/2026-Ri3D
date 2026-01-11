package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.commands.RunStateMachineCommand;
import frc.robot.stateMachines.SuperStateMachine;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AutoManager {

  public record Auto(String name, Supplier<Command> command) {}

  private final SuperStateMachine stateMachine;
  private final Drive drive;
  private final SuperStructure superStructure;

  public AutoManager(SuperStateMachine stateMachine, Drive drive, SuperStructure superStructure) {
    this.stateMachine = stateMachine;
    this.drive = drive;
    this.superStructure = superStructure;
  }

  public SendableChooser<Supplier<Command>> getAutos() {
    SendableChooser<Supplier<Command>> autos = new SendableChooser<>();
    autos.setDefaultOption("DoNothing", Commands::none);
    autos.addOption("ABCD", () -> new RunStateMachineCommand(this::auto_ABCD));
    return autos;
  }

  private StateMachine auto_ABCD() {
    return new AutoStateMachine(superStructure, stateMachine, drive);
  }
}
