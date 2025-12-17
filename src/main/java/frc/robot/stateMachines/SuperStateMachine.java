package frc.robot.stateMachines;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team2930.StateMachine;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.StructureState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class SuperStateMachine {
    public enum SuperState{
        Idle,
        Stow,
        Intake,
        Score,
        ClimbPrep,
        Climb,
        AutoIntake,
        AutoScore
    }

    private SuperState prevState = SuperState.Idle;
    private SuperState state = SuperState.Idle;
    private boolean newState;

    private final Drive drive;
    private final SuperStructure superStructure;
    private final Elevator elevator;
    private final Arm arm;
    private final Intake intake;

    private StateMachine currentMachine;

    public SuperStateMachine(Drive drive, SuperStructure superStructure, Elevator elevator, Arm arm, Intake intake){
        this.drive = drive;
        this.superStructure = superStructure;
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
    }

    public void periodic(){
        newState = state != prevState;
        switch (state) {
            case Idle:
                superStructure.setState(StructureState.Idle);
                break;
            case Stow:
                superStructure.setState(StructureState.Stow);
                break;
            case Intake:
                runStateMachine(() -> new IntakeStateMachine(), newState);
                break;
            case Score:
                runStateMachine(() -> new ScoreStateMachine(), newState);
                break;
            case ClimbPrep:
                superStructure.setState(StructureState.ClimbPrep);
                break;
            case Climb:
                superStructure.setState(StructureState.Climb);
                break;
            case AutoIntake:
                runStateMachine(() -> new AutoIntakeStateMachine(), newState);
                break;
            case AutoScore:
                runStateMachine(() -> new AutoScoreStateMachine(), newState);
                break;
            default:
                break;
        }
        prevState = state;
    }

    private void runStateMachine(Supplier<StateMachine> machineSupplier, boolean newState){
        if(newState){
            currentMachine = machineSupplier.get();
            currentMachine.init();
        }
        currentMachine.advance();
    }

    public void setState(SuperState state){
        this.state = state;
    }

    public static Command setStateCommand(SuperStateMachine superStateMachine, SuperState state){
        return Commands.runOnce(() -> superStateMachine.setState(state));
    }
}
