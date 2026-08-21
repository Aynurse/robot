
package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SetShooterRPMCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;



/** Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

private final XboxController controller =
    new XboxController(OperatorConstants.kDriverControllerPort);

    private final Joystick keyboard = new Joystick(0);
    private final SendableChooser<Command> autoChooser;


  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final SwerveJoystickCommand swerveJoystickCommand = new SwerveJoystickCommand(swerveSubsystem, controller);
  private final TurnToAngleCommand turnToAngleCommand = new TurnToAngleCommand(swerveSubsystem, 75);
  private final SetShooterRPMCommand setShooterRPMCommand = new SetShooterRPMCommand(shooterSubsystem, 3000);
  private final InstantCommand shooterInstantCommand = new InstantCommand(() -> shooterSubsystem.setTargetRPM(4000));

  private final PIDConstants translationController = new PIDConstants(Constants.SwerveConstants.AUTO_TRANS_kP, Constants.SwerveConstants.AUTO_TRANS_kI, Constants.SwerveConstants.AUTO_TRANS_kD);
  private final PIDConstants rotationController = new PIDConstants(Constants.SwerveConstants.AUTO_ROT_kP, Constants.SwerveConstants.AUTO_ROT_kI, Constants.SwerveConstants.AUTO_ROT_kD);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(swerveJoystickCommand);
    NamedCommands.registerCommand("setShooterRPM", shooterInstantCommand);

     

    // Configure the trigger bindings
    configureBindings();
    configureAutoBuilder();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  public void configureAutoBuilder() {

    AutoBuilder.configure(swerveSubsystem::getPose,
    //burda autobuilderi configre etmemiz lazım ve 8 parametre alıyor
    //ilki robotun sahada nerde olduğu ile alakalı
    //swerveSubsystem:: kısmı bizim bu methodu hangi subsystemden alacağımızı gösteriyor

     swerveSubsystem::resetPose,
     //bu pozisyonun sıfırlanması gerekirse hangi methodu kullanarak sıfırlanacağını gösteriyor

      swerveSubsystem::getCommandedSpeeds,
      //bu hızı nerden alabileceğini söylüyor
       (ChassisSpeeds, DriveFeedforwards) -> swerveSubsystem.drive(ChassisSpeeds), 
       //drive komutunun çalışması için içine parametre atmamız lazım
       //bu değişken iki tane parametre istiyor, ikisini de veriyoruz ama sadece biri kullanıyoru
       //öbürünü görmezden geliyoruz

       new PPHolonomicDriveController(translationController, rotationController),
       //nefertiti cnm bunlar da bir translation (ilerleyiş) bir de rotation pidsi
       //her yerde de pid
       //he bi de pid controller değil constants istiyomuş 

        new RobotConfig(50, 3,
        //motorun değerlerini ayarlıyoruz imdat
        //50 robotun kilosu
        //3 ataleti
        //bunu da choreonun sitesainden hesapladım


        new ModuleConfig(Constants.SwerveConstants.TEKERLEK_YARICAPI, 
        //burda modüllerin konfigleri lazım
        //ilk başta tekerleğin metre cinsinden yarıçapı lazım

        Constants.SwerveConstants.MAX_SPEED, 
        //max hızı kaç olabiliyor

        2.255, 
        //sürtmesi kaç (cddeki elemanlara göre en iyisi bu)

        DCMotor.getKrakenX60(1),
        //hangi motor

        60, 
        //kaç amper güç çekeibliyor 
        1 ),
        //kaç motoru var
         0.60), 

        //verene kadar öldüm galiba

        () -> {
    var alliance = DriverStation.getAlliance();
    //ittifak bilgisini çeker cnm 
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    //allience bulunabiliyor mu diye bakar
    //gelen allienceın rengi kırmızı mı diye kontrol eder
}, 
        //sahanın simetriğini öbür taraf için kullanmalı mıyım
        
       swerveSubsystem);
       //bu da zorunluluk

       System.out.println("PATHPLANNER CONFIGURED: " + AutoBuilder.isConfigured());
  }

  private void configureBindings() {
    new Trigger(controller:: getAButton).
    onTrue(turnToAngleCommand);

    //a butonuna yeni bir trigger atadı
    //a butonuna basınca bu commandi çalıştır

     new Trigger(() -> keyboard.getRawButton(1))
    .onTrue(turnToAngleCommand);

    new Trigger(controller::getXButton)
    .onTrue(new InstantCommand(() ->
        turretSubsystem.setTargetAngle(Constants.SwerveConstants.LEFT_ANGLE)));

    new Trigger(controller::getYButton)
    .onTrue(new InstantCommand(() ->
        turretSubsystem.setTargetAngle(Constants.SwerveConstants.RIGHT_ANGLE)));


        new Trigger(controller:: getBButton)
        .onTrue(setShooterRPMCommand);

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
public Command getAutonomousCommand() {

    System.out.println("AUTO COMMAND ALINDI!");

    Command auto = new PathPlannerAuto("otonom");

    System.out.println("AUTO REQUIREMENTS: " + auto.getRequirements());

    return Commands.sequence(
        Commands.runOnce(() -> System.out.println("!!!!! AUTO COMMAND EXECUTE EDİLDİ !!!!!")),
        auto
    );
}
}
