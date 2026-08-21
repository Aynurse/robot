package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;

public class TurnToAngleCommand extends Command {

    private SwerveSubsystem swerve;
    private double targetAngle;
    private double currentAngle;
    private PIDController pidController;
    private double setPointTime;
    private boolean wasAtSetPoint = false;
    
    //tanımlamaları ypaıyoruzz

    public TurnToAngleCommand(SwerveSubsystem swerve, double targetAngle) {
        this.swerve = swerve;
        this.targetAngle = MathUtil.inputModulus(targetAngle, -180, 180)  ;
        pidController = new PIDController(Constants.SwerveConstants.TURN_kP, Constants.SwerveConstants.TURN_kI, Constants.SwerveConstants.TURN_kD);
        //burda yeni bir pid controller objesi oluşturuyoruz

        addRequirements(swerve);

        //burası da her zamanki gibi constructor
    }

    public void initialize() {
        pidController.enableContinuousInput(-180, 180);
        //bu bizim angle wrap olayımızı düzeltcek eleman
        //angle wrap: motorda 2 dereceden 358e gitmek gerekiyorsa 356 açı dönmek yerien 4 açı dönmesine karar verir
        //Bu değerler bir çizgi üzerinde değil, çember üzerinde

        pidController.setTolerance(2);
        
        pidController.reset();

        wasAtSetPoint = false;
        setPointTime = 0.0;


    }

    public void execute() {
        
        currentAngle = swerve.getRotation().getDegrees();
        //current angle değişkenine pigeondan aldığımız gyroyu koyuyoruzz
        //ardından bunu rotation2den dereceye değiştiriyoruz
        //ÇÜNKÜÜ pid rotation2d kabul etmiyor

       double pidOutput = pidController.calculate(currentAngle, targetAngle);
       //bu targetAngledan currentAnglea ne kadar mesafe var onu hesaplıyorr
       //bu değer bizim robotun dönüş hareketine vereceğimiz kontrol sinyali olacak
       
       pidOutput = MathUtil.clamp(pidOutput, -0.5, 0.5);
       //verilen iki değer arasında bir değer getir

       ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, pidOutput);


       swerve.drive(chassisSpeeds);
    

        }
        
    public boolean isFinished() {

        if (pidController.atSetpoint()) {

            //eğer ki pidcontroller olması gerektiği yerdeyse

            if (!wasAtSetPoint) {
            
                setPointTime = Timer.getTimestamp();
                //başlangıç saatini al 

                wasAtSetPoint = true;
                //şu anda set pointteyim
            }

            double currentTime = Timer.getTimestamp();
            //şu anki zamanı al 

            double difference = currentTime - setPointTime;
            //aradaki farkı bul

            if (difference >= 0.2) {

                //fark 2den azsa
                return true;
            }

        } else {
            wasAtSetPoint = false;
            //robot setpointten çıkarsa
        }

        return false;
    }

    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }

}
