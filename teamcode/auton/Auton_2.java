package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.Delayed;

@Autonomous
public class Auton_2 extends LinearOpMode {

    public DcMotor RMF;    //Porta 0 - Motor Mecanum - Direita Frontal
    public DcMotor RMB;    //Porta 1 - Motor Mecanum - Direita Traseiro
    public DcMotor LMF;    //Porta 2 - Motor Mecanum - Esquerda Frontal
    public DcMotor LMB;    //Porta 3 - Motor Mecanum - Esquerda Traseiro
    public DcMotor MAT;     //Porta 0 - Motor UltraPlanetary - Antebraço
    public DcMotor MSE;     //Porta 2 - Motor UltraPlanetary - Sistema de Elevação
    public DcMotor MRP;     //Porta 3 - Motor Core Hex - Rotação Prato
    public Servo servoGarra;    //Porta 0 - Direita
    public Servo servoGarra2;   //Porta 1 - Esquerda
    private Orientation angles;

    private final double VELOCIDADE_MAXIMA = 0.5;
    private final double TEMPO_PARA_ANDAR = 2.4; // Aproximadamente 120cm / 0.5 (velocidade máxima)
    private final double TEMPO_GIRO = 0.75; // Aproximadamente 45 graus / 0.5 (velocidade máxima)

    private double tempoDeInicio = 0;

    static final double MAX_POS_D     =  0.72;     // Maximum rotational position 0.5
    static final double MIN_POS_D     =  0.95;     // Minimum rotational position    certo

    static final double MAX_POS_E     =  0.43;     // Maximum rotational position
    static final double MIN_POS_E     =  0.12;     // Minimum rotational position 0.5

    double  position_D = MIN_POS_D; // Start at halfway position
    double  position_E = MIN_POS_E; // Start at halfway position



    @Override
    public void runOpMode() throws InterruptedException {
        RMF = hardwareMap.dcMotor.get("RMF");
        RMB = hardwareMap.dcMotor.get("RMB");
        LMF = hardwareMap.dcMotor.get("LMF");
        LMB = hardwareMap.dcMotor.get("LMB");
        MAT = hardwareMap.get(DcMotor.class, "MAT");
        MSE = hardwareMap.get(DcMotor.class, "MSE");
        MRP = hardwareMap.get(DcMotor.class, "MRP");
        servoGarra = hardwareMap.get(Servo.class, "servoGarra");
        servoGarra2 = hardwareMap.get(Servo.class, "servoGarra2");

        LMF.setDirection(DcMotorSimple.Direction.FORWARD);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        MAT.setDirection(DcMotorSimple.Direction.FORWARD);
        MSE.setDirection(DcMotorSimple.Direction.REVERSE);
        servoGarra.setDirection(Servo.Direction.FORWARD);
        servoGarra2.setDirection(Servo.Direction.FORWARD);

        waitForStart();
            double tempoPercorrido =  tempoDeInicio;
            if(tempoDeInicio < 1.7){
//                        Ida
                MRP.setPower(0.3);

                position_D = MAX_POS_D;
                position_E = MAX_POS_E;
                servoGarra.setPosition(position_D);         //Direita
                servoGarra2.setPosition(position_E);        //Esquerda
                Thread.sleep(1000);

                MAT.setPower(VELOCIDADE_MAXIMA + 0.2);
                Thread.sleep(1700);
                MAT.setPower(VELOCIDADE_MAXIMA - 0.1);

                MSE.setPower(-VELOCIDADE_MAXIMA * 2);
                Thread.sleep(2100);
                MSE.setPower(0);

                RMF.setPower(VELOCIDADE_MAXIMA );
                RMB.setPower(VELOCIDADE_MAXIMA );
                LMF.setPower(VELOCIDADE_MAXIMA );
                LMB.setPower(VELOCIDADE_MAXIMA );
                Thread.sleep(1730);

                RMF.setPower(-VELOCIDADE_MAXIMA);//-
                RMB.setPower(-VELOCIDADE_MAXIMA);//-
                LMF.setPower(VELOCIDADE_MAXIMA);//+
                LMB.setPower(VELOCIDADE_MAXIMA);//+
                Thread.sleep(345);

                RMF.setPower(VELOCIDADE_MAXIMA /2 );
                RMB.setPower(VELOCIDADE_MAXIMA /2 );
                LMF.setPower(VELOCIDADE_MAXIMA /2 );
                LMB.setPower(VELOCIDADE_MAXIMA /2 );
                Thread.sleep(1380);
                RMF.setPower(0);
                RMB.setPower(0);
                LMB.setPower(0);
                LMF.setPower(0);

                sleep(1000);


                position_D = MIN_POS_D;
                position_E = MIN_POS_E;
                servoGarra.setPosition(position_D);         //Direita
                servoGarra2.setPosition(position_E);        //Esquerda
                Thread.sleep(2000);

//                           volta
//                RMF.setPower(-VELOCIDADE_MAXIMA /2 );
//                RMB.setPower(-VELOCIDADE_MAXIMA /2 );
//                LMF.setPower(-VELOCIDADE_MAXIMA /2 );
//                LMB.setPower(-VELOCIDADE_MAXIMA /2 );
//                Thread.sleep(1150);
//
//                RMF.setPower(VELOCIDADE_MAXIMA);//+
//                RMB.setPower(VELOCIDADE_MAXIMA);//+
//                LMF.setPower(-VELOCIDADE_MAXIMA);//-
//                LMB.setPower(-VELOCIDADE_MAXIMA);//-
//                Thread.sleep(300);
//                RMF.setPower(0);
//                RMB.setPower(0);
//                LMB.setPower(0);
//                LMF.setPower(0);
//
//                RMF.setPower(VELOCIDADE_MAXIMA);//+
//                RMB.setPower(VELOCIDADE_MAXIMA);//+
//                LMF.setPower(-VELOCIDADE_MAXIMA);//-
//                LMB.setPower(-VELOCIDADE_MAXIMA);//-
//                Thread.sleep(850);
            }
        }
    }