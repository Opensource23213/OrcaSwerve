package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="RoboTest", group="ABC Opmode")
public class RoboTest extends OpMode {
    public DcMotorEx spindexer;
    public DcMotorEx intake;
    public Servo flippy;
    public Servo blocker;
    public static double up = .3;
    public static double down = .18;
    public static double x = .26;
    public static double y = .2;
    public static double l1 = up + .05;
    public static double r1 = down;
    public static double block = .05;
    public static double unblock = .29;

    public double test_pos = up;
    public boolean blocking = false;
    public Servo flap;
    public DcMotorEx shoot1;
    public DcMotorEx shoot2;
    public static double speed = 0;
    public static PIDFCoefficients pidf = new PIDFCoefficients(80, 20, 0, 1);
    @Override
    public void init(){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        flippy = hardwareMap.get(Servo.class, "flippy");
        blocker = hardwareMap.get(Servo.class, "blocker");
        flap = hardwareMap.get(Servo.class, "flap");
        shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop(){
        intake.setPower(gamepad1.left_stick_y);
        spindexer.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        if(gamepad1.a){
            test_pos = up;
            blocking = false;
        }else if(gamepad1.b){
            test_pos = down;
            blocking = true;
        }else if(gamepad1.x){
            test_pos = x;
        }else if(gamepad1.y){
            test_pos = y;
        } else if(gamepad1.left_bumper){
            blocking = false;
            test_pos = l1;
        }else if(gamepad1.right_bumper){
            test_pos = r1;
            blocking = false;
        }
        flippy.setPosition(test_pos);
        if(blocking){
            blocker.setPosition(block);
        }else{
            blocker.setPosition(unblock);
        }
        shoot2.setVelocity(speed);
        shoot1.setVelocity(speed);
        flap.setPosition(.04);
        telemetry.update();
    }
}
