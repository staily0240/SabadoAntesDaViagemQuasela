package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Date;
import java.util.Locale;
import java.util.Timer;

@TeleOp
public class TestandoTestes extends LinearOpMode {
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;

    boolean gravar = false;
    boolean gravar2 = false;
    double valorArmazenado[][] = new double[500][5];
    int iGravacao = 0;
    int jGravacao = 0;
    double speed = 1;
    double angulofinalE;
    double angulofinalD;
    double angulorealE = 0;
    double angulorealD = 0;
    final double PI = Math.PI;
    double fMD = 0;
    double fME = 0;
    double forcax = 0;
    double forcax2 = 0;
    double forcay = 0;
    ElapsedTime timer3 = new ElapsedTime();
    double timer5 = 0;
    double forca, pointBraco;
    double servoLagosta = 0;
    double  MIN_PALETA = 0.2;
    double  MAX_PALETA = 0.45;
    double  speedPaleta = 0.05;
    double proporcional =0, integral = 0, derivada = 0, ultErro = 0;
    int posicion = 0;
    boolean controle = false;

    double controleDoProblema = 0;


    BNO055IMU imu;

    Orientation angles;


    @Override public void runOpMode() {
        boolean qqw = true;
        double pointBraco = 0;
        double powerBraco;
        String nivel = "Coletar";
        double switchAngle = 0;
        boolean c = false;
        double speedBraco = 0.5;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");

        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

//        MB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ML.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        timer3.time();
        while(opModeIsActive()){

            boolean povU = gamepad2.dpad_up;          //Seta - Cima
            boolean povR = gamepad2.dpad_right;       //Seta - Direita
            boolean povL = gamepad2.dpad_left;        //Seta - Esquerda
            boolean povD = gamepad2.dpad_down;        //Seta - Baixo

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;
            double y2 = -gamepad1.right_stick_y;
            double d = Math.hypot(x,y);
            double d2 = Math.hypot(x2, y2);
            double angulorad = (d!=0)?Math.asin(y/d):0;
            double angulo = Math.toDegrees(angulorad);
            double angulorad2 = (d2!=0)?Math.asin(y2/d2):0;
            double anguloD = Math.toDegrees(angulorad2);
            boolean r1 = gamepad1.right_bumper;
            boolean l1 = gamepad1.left_bumper;
            double anguloRobo;

//            if (digitalTouch.getState() == true) {
//                telemetry.addData("Digital Touch", "Is Not Pressed");
//            } else {
//                telemetry.addData("Digital Touch", "Is Pressed");
//            }

//========================================== CONTROLE - 1 ==========================================

            //Control Speed
            speed = gamepad1.b ? 1 : gamepad1.a ? 0.5 : gamepad1.x ?0.25:speed ;

//            if (gamepad1.dpad_up){
//                setPowerGravado();
//            }
            switchAngle = gamepad1.dpad_down ? getAngle() : switchAngle;

            // Pegar ângulo Joy Esquerdo
            //1º QUADRANTE
            if((angulo >= 0) && (angulo <= 90) && (x >= 0)) {
                angulofinalE = angulo;
            }
            //2º QUADRANTE
            else if ((angulo >= 0) && (angulo <= 90) && (x <= 0)) {
                angulofinalE = 180-angulo;
            }
            //3º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x <= 0)) {
                angulofinalE = -angulo-180;
            }
            //4º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x >= 0)) {
                angulofinalE = angulo;
            }

            // Pegar ângulo Joy Direito
            //1º QUADRANTE
            if((x2 >= 0 ) && (y2 >= 0)){
                angulofinalD = anguloD;
            }
            // 2º QUADRANTE
            else if((x2 < 0 ) && (y2 >= 0)) {
                angulofinalD = 180-anguloD;
            }
            // 3º QUADRANTE
            else if((x2 < 0 ) && (y2 < 0)){
                angulofinalD = -anguloD-180;
            }
            // 4º QUADRANTE
            else if((x2 >= 0) && (y2 < 0)) {
                angulofinalD = anguloD;
            }
            // MOVIMENTAÇÃO POR ÂNGULO

            anguloRobo = getAngle();
            //Analógico Esquerdo
            angulorealE =  angulofinalE - anguloRobo + switchAngle;
            forcay = Math.sin(Math.toRadians(angulorealE)) * d;
            forcax = Math.cos(Math.toRadians(angulorealE)) * d;

            //Analógico Direito
            angulorealD =  angulofinalD - anguloRobo + switchAngle;
            forcax2 = Math.cos(Math.toRadians(angulorealD)) * d2;

            //1º QUADRANTE FORÇA

            angulorad = (d!=0)?Math.asin(forcay/d):0;

            if((forcax>= 0) && (forcay>=0)) {
                fME = d;
                fMD = (((4 / PI) * angulorad) - 1) * d;

            }
            //2º QUADRANTE FORÇA
            else if ((forcax< 0) && (forcay>=0)) {
                fME = (((4 / PI) * angulorad) - 1) * d;
                fMD = d;
            }
            //3º QUADRANTE FORÇA
            else if ((forcax< 0) && (forcay<0)) {
                fME = -d;
                fMD = (((4 / PI) * angulorad) + 1) * d ;
            }
            //4º QUADRANTE FORÇA
            else if ((forcax>= 0) && (forcay<0)) {
                fME = (((4 / PI) * angulorad) + 1) * d;
                fMD = -d;
            }

            RMF.setPower((fMD-x2) * speed);
            LMF.setPower((fME+x2) * speed);
            RMB.setPower((fME-x2) * speed);
            LMB.setPower((fMD+x2) * speed);
        }
    }

    public double getAngle(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

}


