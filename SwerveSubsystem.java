package frc.robot.subsystems;

import frc.robot.Constants;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final SwerveDriveOdometry swerveDriveOdometry;
    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSim;
    private final Field2d field;
    private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
    private double lastSimTime;

    public SwerveSubsystem () {

    frontLeft = new SwerveModule(Constants.SwerveConstants.FRONT_LEFT_DRIVE_ID, 
    Constants.SwerveConstants.FRONT_LEFT_STEER_ID, 
    Constants.SwerveConstants.FRONT_LEFT_CANCODER_ID, 0);

    //Burda hepsinin idlerini tanımlıyoruz yeni obje oluşturarak
    //Bundan dört tanr yapıyoruz

    
    frontRight = new SwerveModule(Constants.SwerveConstants.FRONT_RIGHT_DRIVE_ID, 
    Constants.SwerveConstants.FRONT_RIGHT_STEER_ID, 
    Constants.SwerveConstants.FRONT_RIGHT_CANCODER_ID, 0);


    backLeft = new SwerveModule(Constants.SwerveConstants.BACK_LEFT_DRIVE_ID, 
    Constants.SwerveConstants.BACK_LEFT_STEER_ID, 
    Constants.SwerveConstants.BACK_LEFT_CANCODER_ID, 0);


    backRight = new SwerveModule(Constants.SwerveConstants.BACK_RIGHT_DRIVE_ID, 
    Constants.SwerveConstants.BACK_RIGHT_STEER_ID, 
    Constants.SwerveConstants.BACK_RIGHT_CANCODER_ID, 0);

    Translation2d frontLeftLocation = new Translation2d(0.30, 0.30);
    //bunlar swervelerin merkeze göre hangi konumda olduğunu gösteriyor
    //y sağ sol x ön arka


    Translation2d frontRightLocation = new Translation2d(0.30, -0.30);

    Translation2d backLeftLocation = new Translation2d(-0.30, 0.30);

    Translation2d backRightLocation = new Translation2d(-0.30, -0.30);

    swerveDriveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    //bunda da yine tanımlama yapıoz obje oluşturuyoz

    gyro = new Pigeon2(Constants.SwerveConstants.PIGEON_2_ID);
    //pigeonun idsini atadım

gyroSim = gyro.getSimState();

    SwerveModulePosition[] modulePositions = {
    frontLeft.getModulePosition(),
    frontRight.getModulePosition(),
    backLeft.getModulePosition(),
    backRight.getModulePosition()

    //burda liste halinde tüm modüllerin pozisyonunu alıyorum
};

    swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, getRotation(), modulePositions);
    //yeni obje oluştuuryom

    field = new Field2d();
    SmartDashboard.putData(field);

    lastSimTime = Timer.getFPGATimestamp();
    

    }

    public void drive (ChassisSpeeds chassisSpeeds) {
        //drive methodunu oluşturuyoruz

        commandedSpeeds = chassisSpeeds;

        SwerveModuleState swerveModuleState[] = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        //burda kinematicsi module statee çeviriyoruz cnm
        //değişkenimiz dışardan aldığımız chassis speeds
        //bize bilgi array olarak geliyor

        SwerveModuleState frontLeftState = swerveModuleState[0];
        SwerveModuleState frontRightState = swerveModuleState[1];
        SwerveModuleState backLeftState = swerveModuleState[2];
        SwerveModuleState backRightState = swerveModuleState[3];
        //burda da statelerin hangi arrayle associatelendiklerini yazıyoruz
        //valla türkçesini hatırlamıom yoksa kullanırdım
        
        frontLeft.setDesiredState(frontLeftState);
        frontRight.setDesiredState(frontRightState);
        backLeft.setDesiredState(backLeftState);
        backRight.setDesiredState(backRightState);
        //burda da setdesiredstate değişkenimizi kullanarak teker teker modüllere veriyoruz istediğimiz açı ve hızı

        System.out.println(
    "AUTO DRIVE: vx=" + chassisSpeeds.vxMetersPerSecond
    + " vy=" + chassisSpeeds.vyMetersPerSecond
    + " omega=" + chassisSpeeds.omegaRadiansPerSecond
);

    }

    public Rotation2d getRotation() {

        return gyro.getRotation2d();
        //burda gyronun açısını alıyoruz

    }

        @Override
        public void periodic() {


            frontLeft.updateSimulator();
            frontRight.updateSimulator();
            backLeft.updateSimulator();
            backRight.updateSimulator();

            if (RobotBase.isSimulation()) {

    double currentSimTime = Timer.getFPGATimestamp();
    double dt = currentSimTime - lastSimTime;
    lastSimTime = currentSimTime;
    

    gyroSim.addYaw(
        Math.toDegrees(
            commandedSpeeds.omegaRadiansPerSecond * dt
        )
    );
}

            //odometriye koymak için swervelerin nerde oldğuunu ve pigeonun gösterdiği açıyı alıyoruz ve birleştiriyoruz
            Rotation2d gyroPosition = getRotation();



            SwerveModulePosition[] modulePositions = {
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            backLeft.getModulePosition(),
            backRight.getModulePosition()
            };

            swerveDriveOdometry.update(gyroPosition, modulePositions);

            field.setRobotPose(getPose());


}

    public Pose2d getPose() {
        return swerveDriveOdometry.getPoseMeters();
        //Burda odometride nerd eoldğuunu alıyoruz..
    }
    
public void resetPose(Pose2d pose) {

    gyro.setYaw(pose.getRotation().getDegrees());

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
        frontLeft.getModulePosition(),
        frontRight.getModulePosition(),
        backLeft.getModulePosition(),
        backRight.getModulePosition()
    };

    swerveDriveOdometry.resetPosition(
        getRotation(),
        modulePositions,
        pose
    );
}

    public ChassisSpeeds getCommandedSpeeds() {
        return commandedSpeeds;
    }
}

