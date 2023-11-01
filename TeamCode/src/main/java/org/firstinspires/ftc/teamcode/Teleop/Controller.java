package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Controller extends OpMode{

    private final ElapsedTime runtime = new ElapsedTime();

    //declaring motes

    private DcMotorEx wheelFL;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBR;

    private DcMotor intakeRight;
    private DcMotor intakeLeft;


    private double previousRunTime;
    private double rotation;

    //intake on bool
    private boolean intakeOn;
    private boolean aPressed;



public void init(){
 telemetry.addData("Status", "Initialized");


 //setting moters to where they are in driver controller config.
    wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
    wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
    wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
    wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");

    //adding intake moters.
    intakeRight = hardwareMap.get(DcMotor.class, "leftInta");
    intakeLeft = hardwareMap.get(DcMotor.class, "RightInta");

    //allowing it to run withought encoders
    wheelFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    wheelFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    wheelBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    wheelBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




    //setting makes some moters reversed due to their diffrent orintations of the robot.
    wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
    wheelFR.setDirection(DcMotorSimple.Direction.FORWARD);
    wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);
    wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

    intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
    intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);

    //tell driver initialization if complete - do not add vulgarity @isaiah @saiansh
    telemetry.addData("Status","Initialized");
}
    @Override
    public void init_loop(){} //runs after init but berfore play

    public void start(){
    runtime.reset();
    previousRunTime = getRuntime(); //mabey a bug but found it like this
    }



    @Override
    public void loop(){
        basicDrive();
        intake();
    }

    public void basicDrive(){
    //sets the speed of the robot based on how far the joystick is pushed
        double setSpeed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

        double moveAngle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) - Math.PI / 4;
        double turnAmount = gamepad1.right_stick_x;
        rotation += 1 * turnAmount;

        final double fLPower = setSpeed * (Math.cos(moveAngle  + 90)) - turnAmount;
        final double fRPower = setSpeed * (Math.sin(moveAngle  + 90)) + turnAmount;
        final double bLPower = setSpeed * (Math.sin(moveAngle  + 90)) - turnAmount;
        final double bRPower = setSpeed * (Math.cos(moveAngle  + 90)) + turnAmount;


        wheelFL.setPower(-fLPower);
        wheelFR.setPower(-fRPower);
        wheelBL.setPower(bLPower);
        wheelBR.setPower(bRPower);

    }

    public void intake(){
        if(gamepad1.a){
            
            if(aPressed){

            } else{
                aPressed = true;
                intakeOn = !intakeOn;
            }
        } else {
            aPressed = false;
        }

        if(intakeOn){
            intakeRight.setPower(1);
            intakeLeft.setPower(1);
        } else {
            intakeRight.setPower(0);
            intakeLeft.setPower(0);
        }
    }


    public static void wait(int ms) {
        try {
            Thread.sleep(ms); //core java delay command
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt(); //this exception is useful to remove the glitches and errors of the thread.sleep()
        }
    }

    //@Override
    void Stop(){
    }








}
