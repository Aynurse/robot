package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;



public class SwerveJoystickCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final XboxController controller;

public SwerveJoystickCommand(
    SwerveSubsystem swerveSubsystem,
    XboxController controller
) {
    this.swerveSubsystem = swerveSubsystem;
    this.controller = controller;

    addRequirements(swerveSubsystem);
}




    @Override
    public void execute() {

        System.out.println("JOYSTICK DRIVE");

        double x = -controller.getRawAxis(1);
        double y = controller.getRawAxis(0);
        double rx = controller.getRawAxis(4);
        //controllerdan joystick değerlerini aldık


        //DEADZONE

                if (Math.abs(x) < 0.1) {
            x = 0;
        }

        if (Math.abs(y) < 0.1) {
            y = 0;
        }

        if (Math.abs(rx) < 0.1) {
            rx = 0;
        }



        double xspeed = x * Constants.SwerveConstants.MAX_SPEED;
        double yspeed = y * Constants.SwerveConstants.MAX_SPEED;
        double rxspeed = rx * Constants.SwerveConstants.MAX_ANGULAR_SPEED;
        //burda da şasenin max hızıyla çarpıyoruz...

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xspeed, yspeed, rxspeed);
        //bu değişkenlerle yeni bir obje oluşturuyoruz

        swerveSubsystem.drive(chassisSpeeds);
        //subsystemin içine çağırıyoruz


        
        }
    }


