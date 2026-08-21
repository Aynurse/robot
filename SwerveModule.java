package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
//ben yeminlen bu işi bırakcam imdat


public class SwerveModule {

    //swerve modülleirnden tek bir tanesini tanımladığımız var

    private final TalonFX driveMotor;
    private final TalonFXSimState driveSimMotor;
    private final TalonFX steerMotor;
    private final TalonFXSimState steerSimMotor;
    private final CANcoder canCoder;
    private final CANcoderSimState canCoderSim;
    private final double encoderOffset;
    private final int canCoderID;
    private final DCMotorSim driveMotorSim;
    private final DCMotorSim steerMotorSim;
    //simülasyonda kullanmak için sanal motor oluşturuyoruz...

    public SwerveModule(
        int driveMotorID,
        int steerMotorID,
        int canCoderID,
        double encoderOffset
    ) {

        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(steerMotorID);
        canCoder = new CANcoder(canCoderID);
        driveSimMotor = driveMotor.getSimState();
        steerSimMotor = steerMotor.getSimState();
        canCoderSim = canCoder.getSimState();
        driveMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            //motor modeli
            0.025,
            //atalet bişey diyo da bilmiom
            //sistemin dönmeye karşı ne kadar direnç gösterdiğini temsil eden fiziksel değer.
            1.0
            //orantı
    ),
    DCMotor.getKrakenX60(1)
    //ir adet kraken x60 simüle ediyorum
);
        steerMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.025, Constants.SwerveConstants.STEER_GEAR_RATIO),
    DCMotor.getKrakenX44(1));
        this.encoderOffset = encoderOffset;
        this.canCoderID = canCoderID;

        // motor = new variabletype(id name);
        //thisle soldaki sınıf değişkeni sağdaki parametre diyoruz. isim benzerliğinde kullanılır. 

        configureHardware();

        //motorların ayarlanmasını sağlar

    }

    private void configureHardware() {

        configureDriveMotor();
        configureSteerMotor();
        configureCANcoder();

        //burası aşağıda yazdığımız methodları içinde toplar ve 

        }

    private void configureDriveMotor() {

        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
        //obje oluşturanzi...

        driveConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //bu motorun saat yönünü pozitif olarak göreceği anlamına geliyor

        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //Bu robotun dururken brake modunda duracağını belirliyor 

        driveConfiguration.Slot0.kP = Constants.SwerveConstants.DRIVE_kP;
        driveConfiguration.Slot0.kD = Constants.SwerveConstants.DRIVE_kD;
        //driveın pidlerini belirlioz...

        driveMotor.getConfigurator().apply(driveConfiguration);
        //drive motoruna configleyiciyle beraber bu özellikleri tanımla
    }

    private void configureSteerMotor() {

        TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();

        steerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        steerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        steerConfiguration.Slot0.kP = Constants.SwerveConstants.STEER_kP;
        steerConfiguration.Slot0.kD = Constants.SwerveConstants.STEER_kD;

        //slot0 bizim pid değerlerini tuttuğumuz "değişken" 

        steerConfiguration.Feedback.withFeedbackRemoteSensorID(canCoderID);
        //encoderın hangi cancoderı base almasını istiyorsun ona karar veriliyor.

        steerConfiguration.Feedback.withSensorToMechanismRatio(Constants.SwerveConstants.STEER_SENSOR_RATIO);
        //bu cancoderdan mekanizmaya dönüşümü temsil ediyor

        steerMotor.getConfigurator().apply(steerConfiguration);
    }

    private void configureCANcoder() {

        CANcoderConfiguration caNcoderConfiguration = new CANcoderConfiguration();


        caNcoderConfiguration.MagnetSensor.MagnetOffset = encoderOffset;
        //CANcoder'ın okuduğu manyetik açıyı gerçek mekanik sıfıra çevirmek için kullanılan düzeltme değeridir.
        //her swerve modülünün offseti farklı olabileceği için constructora eklenir.

        caNcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        //bu da cancoderın saat yönünü pozitif alacağını söylüyor cnm


        canCoder.getConfigurator().apply(caNcoderConfiguration);

    }

    public void setDesiredState(SwerveModuleState state) {
        //swerveleri kontrol eden bir modül 
        Rotation2d currentAngle = getAngle();
        //currentangle değişkenine get angledan aldığımız bilgiyi koyuyoruz..
        state.optimize(currentAngle);
       SwerveModuleState optimized = state;
       //öbür optimized şekli 2023de kaldırılmış en yenisi bu cnm

       Rotation2d targetAngle = optimized.angle;
       //target angleı yukarıda belirlediğimiz gibi alıyoruz cnm....

       setAngle(targetAngle);
       //maşallah kızımızın on parmağında da sıfır marifet (üç aslında btw mantı yapabiliyor sarma sarabiliyor.. üçüncüyü unuttm)

       setDriveSpeed(optimized.speedMetersPerSecond);



    }

    public Rotation2d getAngle() {

        double value = canCoder.getAbsolutePosition().getValueAsDouble();
        //cancoder bize açı değil rotasyon verir (0 ile 1 arası bir değer)
        //getValueAsDouble ise bu rotasyonu bir sayıymış gibi alıp kullanmamızı sağlar

        Rotation2d angle = Rotation2d.fromRotations(value);
        //burda ise bu rotasyonu anglea çeviriyoruz

        return angle;

    }

    public void setAngle(Rotation2d angle) {
        //adı belli bence ya aynur

        //double targetWheelRotation = angle.getRotations();
        //bunla tekerlek için belirlenen hedef açıyı rotasyona çeviriyoruz...

        //double targetMotorRotation = targetWheelRotation * Constants.SwerveConstants.STEER_GEAR_RATIO;
        //burda da istenilen motor rotasyonauna ulaşmak için ratioyla hedef rotasyonu çarpıyoruz..

        //steerMotor.setControl(new PositionVoltage(targetMotorRotation));
        //position voltage = Bu pozisyonu hedef olarak al ve PID kullanarak oraya ulaş.
        //setcontrol da ilerletiyo işte...

       double targetRotation = angle.getRotations() * Constants.SwerveConstants.STEER_GEAR_RATIO;

       steerMotor.setControl(new PositionVoltage(targetRotation));
    }
    public void setDriveSpeed(double desiredMetersPerSecond) {

        double wheelRPS = desiredMetersPerSecond / Constants.SwerveConstants.TEKERLEK_CEVRESI;
        //rps = revolutions per second, saniyedeki devir sayısı
        //tekerleğin devir sayısını bulmak için istenilen metreyle tekerleğin çevresini bölüyoruz..
        //ingini hatırlayamadığım için bu değişken türkçe kalcak

    if (Math.abs(desiredMetersPerSecond) < 0.1) {
        driveMotor.stopMotor();
        return;
    }

        double motorRPS = wheelRPS * Constants.SwerveConstants.DRIVE_GEAR_RATIO;
        //burda da motorun devirini bulabilmek için tekerleğin deviri ile orantıyı çarpıyoruz
        //ben r1e göre hesapladım ama değiştirilebilir.

        driveMotor.setControl(new VelocityVoltage(motorRPS));
        //Burda da drivemotorun hızını veriyoruz..
    }

    public double getSpeedMetersPerSecond() {
        StatusSignal<AngularVelocity> currentMotorRPS = driveMotor.getVelocity();
        //öncelikle motorla ilgili herhangi bir şey alırken bu sinir bozucu değişken tipiyle karşılaşacakmışım..
        //ama getValueAsDouble'la beraber normal bir işlem yapaibliyorm...
        //He birde burda motorun hızını alıp değişkene atıom 

        double currentWheelRPS = currentMotorRPS.getValueAsDouble() / Constants.SwerveConstants.DRIVE_GEAR_RATIO;
        //motor devirini gear ratioya bölüp wheelrpsi buluom

        return currentWheelRPS * Constants.SwerveConstants.TEKERLEK_CEVRESI;
        //tekerlek çevresiyle de deviri çarparak da saniyede ilerlediği metre sayını buluom
        //iyne değişkene atardım da üşendim tbh
    }

    //öncelikle belirtmek isterim ki 200. satırımdayım... hiçbişi yazmış gibi hissetmiyorm
    //bu konuda yardımcı olan herşeye teşekkr ederm...

    public double getPosition() {
       StatusSignal<Angle> currentMotorRotation = driveMotor.getPosition();
       //öncelikle rotasyon angle alıyomuş bunu yarım saat aramış bulunmaktayım..
       //burdan toplam ne kadar motor dönmüş onu alıp değişkene atıom

       double currentWheelRotation = currentMotorRotation.getValueAsDouble() / Constants.SwerveConstants.DRIVE_GEAR_RATIO;
       //burda da motor rotasyonunu tekerlek rotasyonuna çevirmek için gear ratioya bölüom

       return currentWheelRotation * Constants.SwerveConstants.TEKERLEK_CEVRESI;
       //burda da kaç metre gittiğini hesaplamak için tekerlek rotasyonuyla tekerleğin çevresini çarpıom

    }

    public SwerveModulePosition getModulePosition() {
        //hayatımda bu kadar saçma az şey görmüşümdür
        //odometri için ben şu kadar yol gittim ve şu açıdayım demek

        double distanceMeters = getPosition();
        //değişkene koydum..

        Rotation2d angle = getAngle();
        //aynısının laciverti 

        return new SwerveModulePosition(distanceMeters, angle);
        //bu yeni objeyi de içindeki verilerle döndürdüm
    }

public void updateSimulator() {

    //bu da simülasyonu kullanmak için yazdığımız method..
    double simMotorVoltage = driveSimMotor.getMotorVoltage();
    //motorun istediği voltaj alınır

    driveMotorSim.setInputVoltage(simMotorVoltage);
    //bu sanala çevrilir

    driveMotorSim.update(0.02);
    //Sanal motoru 20 ms ileri simüle ediyorsun.

    double rawRotation = driveMotorSim.getAngularPositionRotations();
    driveSimMotor.setRawRotorPosition(rawRotation);
    //"Motor şu kadar rotasyon yaptı."


    double rawSpeedRotation = driveMotorSim.getAngularVelocityRPM();
    rawSpeedRotation /= 60;
    driveSimMotor.setRotorVelocity(rawSpeedRotation);
    //Motor saniyede şu kadar rotation yapıyor.


    // TUŞ BIRAKILDIYSA MOTORU ANINDA DURDUR
    if (Math.abs(simMotorVoltage) < 0.01) {
        driveMotorSim.setInputVoltage(0);
        driveMotorSim.setAngularVelocity(0);
        driveSimMotor.setRotorVelocity(0);
    }


    // =========================
    // STEER SIMULATION
    // =========================

    double simSteerVoltage = steerSimMotor.getMotorVoltage();

    steerMotorSim.setInputVoltage(simSteerVoltage);
    steerMotorSim.update(0.02);

    // DCMotorSim bize mekanizma tarafındaki açıyı veriyor
    double steerRotation =
        steerMotorSim.getAngularPositionRotations();

    // TalonFX rotor tarafına geri yazıyoruz
    double steerMotorRotation =
        steerRotation * Constants.SwerveConstants.STEER_GEAR_RATIO;

    steerSimMotor.setRawRotorPosition(steerMotorRotation);

    // CANcoder TEKERLEĞİN açısını ölçüyor.
    // BURADA ARTIK TEKRAR GEAR RATIO İLE BÖLMÜYORUZ.
    canCoderSim.setRawPosition(steerRotation);
}
}
