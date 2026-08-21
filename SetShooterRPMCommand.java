package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterRPMCommand extends Command {

    private ShooterSubsystem shooter;
    private double targetRPM;

    public SetShooterRPMCommand(ShooterSubsystem shooter, double targetRPM) {

        this.shooter = shooter;
        this.targetRPM = targetRPM;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetRPM(targetRPM);
    }
    
    @Override
    public void execute() {
        shooter.setTargetRPM(targetRPM);


    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
