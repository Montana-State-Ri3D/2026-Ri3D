package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team2930.StateMachine;
import frc.lib.team2930.commands.RunStateMachineCommand;
import frc.robot.autonomous.AutoDescriptor.AutoAction;
import frc.robot.autonomous.AutoDescriptor.StartingLocation;
import frc.robot.stateMachines.SuperStateMachine;
import frc.robot.subsystems.SuperStructure;
import java.util.ArrayList;
import java.util.function.Supplier;

public class AutoManager {

  public record Auto(String name, Supplier<Command> command) {}

  private final SuperStateMachine stateMachine;
  private final SuperStructure superStructure;

  public AutoManager(SuperStateMachine stateMachine, SuperStructure superStructure) {
    this.stateMachine = stateMachine;
    this.superStructure = superStructure;
  }

  public SendableChooser<Supplier<Command>> getAutos() {
    SendableChooser<Supplier<Command>> autos = new SendableChooser<>();
    autos.setDefaultOption("DoNothing", Commands::none);
    autos.addOption("ABCD", () -> new RunStateMachineCommand(this::auto_ABCD));
    return autos;
  }

  private StateMachine auto_ABCD() {
    ArrayList<AutoAction> actions = new ArrayList<>();
    actions.add(AutoAction.S);
    actions.add(AutoAction.IA);
    actions.add(AutoAction.S);
    actions.add(AutoAction.IB);
    actions.add(AutoAction.S);
    actions.add(AutoAction.IC);
    actions.add(AutoAction.S);
    actions.add(AutoAction.ID);
    actions.add(AutoAction.S);
    AutoDescriptor descriptor = new AutoDescriptor(StartingLocation.S1, actions);
    return new AutoStateMachine(descriptor, stateMachine, superStructure);
  }
}
