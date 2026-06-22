package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.OrcaAuto;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
//@Disabled
public class DecodeLibrary extends OpMode {
    public static double color = 0;
    public ElapsedTime slow = new ElapsedTime();
    public GoBildaPinpointDriver pin;
    public ElapsedTime fast = new ElapsedTime();
    public double robot_x = 0;
    public double robot_y = 0;
    public double robot_heading = 0;
    public boolean manual_turret = false;
    public boolean first_count = false;
    public static Pose2D auto_pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    public double old_dis = 0;
    public intake intake_system = new intake();
    public shooter shooter = new shooter();
    public turret turret = new turret();
    public ElapsedTime flip_up = new ElapsedTime();
    public static double shoot_power_offset = 1320;
    public static double shoot_multiplier = 220;
    public boolean teleop = true;
    public double index_steps = 0;
    public List<LynxModule> hubs = null;
    public  webcam cam = new webcam();
    public static double pattern = 1;
    public static double anglex_mod = 60;
    public static double x_change = -6;
    public OrcaAuto drive = new OrcaAuto();
    public CameraCode cameraCode = new CameraCode();
    @Override
    public void init(){
        initialize();
    }

    public void initialize() {
        x_mod = -5;
        y_mod = -4;
        drive.SwerveOrcaInit(hardwareMap, gamepad1, color);
        shooter.initialize();
        turret.initialize();
        intake_system.init();
        cameraCode.init();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public void short_match(){
        shoot_power_offset = 1270;
        shoot_multiplier = 220;
    }

    @Override
    public void loop() {
    }
    public Button1 button1 = new Button1();
    public static double gate_angle = 23;
    public double speed_add = 0;
    public class Button1{
        List<String> button = new ArrayList<>();
        List<String> nowbutton = new ArrayList<>();
        List<String> lastbutton = new ArrayList<>();
        String type = "";
        public void button(){
            ButtonControl();
        }
        public void ButtonControl(){
            if(gamepad1.rightBumperWasReleased()) {
                drive.update_pid();
                drive.hold_angle = true;
                if(color == 1){
                    drive.angle_to_hold = 180 - gate_angle;
                }else{
                    drive.angle_to_hold = gate_angle;
                }
            }
            if(abs(gamepad1.right_stick_x) > .05){
                drive.hold_angle = false;
            }
            if(gamepad2.touchpadWasPressed()){
                manual_turret = true;
            }
            if(gamepad1.dpadRightWasReleased() || gamepad2.dpadRightWasReleased()){
                if(color == 0) {
                    y_mod -= 1;
                }else{
                    y_mod += 1;
                }
            }else if(gamepad1.dpadLeftWasReleased() || gamepad2.dpadLeftWasReleased()){
                if(color == 1) {
                    y_mod -= 1;
                }else{
                    y_mod += 1;
                }
            }
            if(gamepad1.dpadUpWasReleased() || gamepad2.dpadUpWasReleased()){
                speed_add += 15;
            }else if(gamepad1.dpadDownWasReleased() || gamepad2.dpadDownWasReleased()){
                speed_add -= 15;
            }
            if(gamepad1.aWasReleased()){
                speed_add = 0;
                x_mod += 123.5 - location.getX();
            }else if(gamepad1.bWasReleased()){
                y_mod += 125.5 - location.getY();
            }else if(gamepad1.psWasReleased()){
                speed_add = 0;
                x_mod += 123.5 - location.getX();
                y_mod += 125.5 - location.getY();
            }

        }

    }


    public boolean robot_going_forward = true;

    public boolean freeze = false;
    public boolean start = false;
    double times = 0;
    boolean back = false;
    public boolean tippy_toe = false;
    public void teleop_loop(){
        if(tippy_toe){
            intake_system.flippy.setPosition(1);
            shooter.shoot1.setVelocity(0);
            shooter.shoot2.setVelocity(0);
            turret.turret_servo_1.setPosition(.5);
            turret.turret_servo_2.setPosition(.5);
            intake_system.intake.setPower(0);
            intake_system.spindexer.setPower(0);
            if(gamepad2.ps){
                tippy_toe = false;
            }
        }else {

            if (start) {
                for (LynxModule hub : hubs) {
                    hub.clearBulkCache();
                }
                shooter.shooting();
                turret.turret_move();
                intake_system.tele();
                button1.button();
                if (shooter.balls_shot < intake_system.balls.size() && gamepad1.left_trigger > .4) {
                    if ((dead_distance * .0254) > 3) {
                        drive.zero = true;
                    } else {
                        drive.zero = true;
                    }

                }
                if (gamepad2.touchpad) {
                    tippy_toe = true;
                }
                drive.runDrive();
            } else if (!gamepad1.atRest()) {
                start = true;
            }else if(drive.pinpoint.getPosition() != auto_pose){
                drive.pinpoint.setPosition(auto_pose);
            }
        }
    }
    public static double alt_flap = .1;
    public static double speed_increase = 350;
    public static double speed_increase2 = 40;
    public static double flap_distance = 2;
    public static double latency = .5;
    public static double close_decrease = 100;
    public class shooter{
        public Servo flap;
        public DcMotorEx shoot1;
        public DcMotorEx shoot2;
        public double speed = 0;
        public double position = 0.05;
        public double current_speed = 0;
        public double set_speed = 0;
        public PIDFCoefficients pidf = new PIDFCoefficients(100, .5, .001, 7);
        public double last_speed = 0;
        public boolean at_speed = false;
        public void initialize(){
            flap = hardwareMap.get(Servo.class, "flap");
            shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
            shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
            shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
            shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void shooting() {
            current_speed = shoot1.getVelocity();
            flap_mod();
            if(gamepad1.left_trigger < .4) {
                set_speed = speed;
                speed = speed + (last_speed - speed) * latency;
            }
            speed = Math.round(speed / 20) * 20;
            shoot2.setVelocity(speed);
            shoot1.setVelocity(speed);
            last_speed = set_speed;
            flap.setPosition(position);
        }
        public double least_speed = 0;
        public double start_speed = 0;
        public double first_press = 0;
        public double balls_shot = 0;
        public double total_balls_shot = 0;

        public void flap_mod(){
            position = .02 - alt_flap  * ((dead_distance * .0254) - flap_distance);
            if(position < .02){
                position = .02;
            }
            double increase = 0;
            if(turret.target_angle - .5 > 0){
                increase = (speed_increase2 - (speed_increase2 * (abs(turret.target_angle - .5 - 90/turret.servo_degrees) / .18)));
            }else{
                increase = -(speed_increase2 - (speed_increase2 * (abs(turret.target_angle - .5 + 90/turret.servo_degrees) / .18)));
            }
            double short_decr = (position - .02) * 10 * close_decrease;

            set_speed = shoot_multiplier * ((dead_distance * .0254) - 1.6);
            set_speed += shoot_power_offset + speed_add;
            set_speed += (speed_increase * abs(turret.target_angle - .5));
            set_speed += increase;
            set_speed -= short_decr;

            if(!intake_system.blocking) {
                if (first_press == 0) {
                    start_speed = current_speed;
                    first_press = 1;
                    least_speed = start_speed;
                    speed = set_speed;
                } else if (first_press == 1) {
                    if (current_speed < least_speed) {
                        least_speed = current_speed;
                    } else if (current_speed > least_speed + 60) {
                        first_press = 2;
                        balls_shot += 1;
                        start_speed = current_speed;
                        least_speed = start_speed;
                    }
                    speed = set_speed;

                } else if (first_press == 2) {
                    speed = set_speed;
                    if (current_speed > start_speed) {
                        start_speed = current_speed;
                        least_speed = current_speed;
                    } else if (current_speed < least_speed) {
                        least_speed = current_speed;
                        first_press = 3;
                    }
                } else if (first_press == 3) {
                    speed = set_speed;
                    if (current_speed < least_speed) {
                        least_speed = current_speed;
                    } else if (current_speed > least_speed + 60) {
                        first_press = 2;
                        balls_shot += 1;
                        start_speed = current_speed;
                        least_speed = start_speed;
                    }
                }
            }else {
                if(balls_shot > 0){
                    intake_system.balls.clear();
                }
                total_balls_shot += balls_shot;
                balls_shot = 0;
                first_press = 0;
                speed = set_speed;
            }

        }
    }
    public static double flippy_up = 0.05;
    public static double flippy_hold = .45;
    public static double  flippy_down = .48;
    public static double mod = 200;
    public static double dis = .8;
    public static double speed_int = -1;
    public static double shot_speed = .35;
    public static double wall_block = .575;
    public static double color_distance = 80;
    public static double sorting_speed = 50;
    public static double spitcount = 2;
    public static double spit_speed = 5;
    public class intake{
        public double flippy_pos = flippy_down;
        public double block = .07;
        public double unblock = .3;
        public boolean blocking = true;
        public boolean spindex_shooting = false;
        public DcMotorEx spindexer;
        public DcMotorEx intake = null;
        public Servo flippy = null;
        public Servo blocker;
        public DigitalChannel disfirst;
        public DigitalChannel dismiddle;
        public DigitalChannel dislast;
        public List<String> balls = new ArrayList<>();
        public AnalogInput spin_dis;
        public boolean outtake = false;
        public boolean shot_right = false;
        public double wall_clear = .25;
        public double wall_pose = wall_block;
        public boolean was_shooting = false;
        public boolean shoot_slower = false;
        public CRServo Light;
        public boolean intake_is_spinning = false;
        public void init(){
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
            spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flippy = hardwareMap.get(Servo.class, "flippy");
            blocker = hardwareMap.get(Servo.class, "blocker");
            Light = hardwareMap.get(CRServo.class, "light");
            Light.setDirection(DcMotorSimple.Direction.REVERSE);
            disfirst = hardwareMap.get(DigitalChannel.class, "disfirst");
            dismiddle = hardwareMap.get(DigitalChannel.class, "dismiddle");
            dislast = hardwareMap.get(DigitalChannel.class, "dislast");
            spin_dis = hardwareMap.get(AnalogInput.class, "spin_dis");
        }
        public void tele(){
            if(gamepad1.left_trigger > .4){
                wall_pose = wall_block;
                if((dead_distance * .0254) > 2.5){
                    spindexer.setPower(-.6);
                }else {
                    spindexer.setPower(-.8);
                }
                flippy_pos = flippy_up;
                blocking = false;
            }else{
                if(balls.size() < 3){
                    flippy_pos = flippy_down;
                }

                blocking = true;
            }
            if(balls.size() == 3 && blocking && wall_pose == wall_block){
                flippy_pos = flippy_up;
                if (stop_intake) {
                    intake.setPower(0);
                }else {
                    intake.setPower(.6);
                }
                spindexer.setPower(0);
            }else {
                if(blocking) {
                    if(intake.getVelocity() < -500 && spit_count == 0){
                        intake_is_spinning = true;
                    }else if((intake.getVelocity() < -spit_speed && intake_is_spinning) || spit_count == 1){
                        spit_out();
                    }
                    spindexer.setPower(1);
                }
                if(spit_count == 0) {
                    intake.setPower(-1 + gamepad1.right_trigger);
                }
            }
            flippy.setPosition(flippy_pos);
            if(blocking){
                blocker.setPosition(block);
            }else{
                spit_count = 0;
                blocker.setPosition(unblock);
            }
            count();
        }
        public boolean stop_intake = false;
        public void auto(){
            if(blocking){
                blocker.setPosition(block);
            }else{
                blocker.setPosition(unblock);
                spit_count = 0;
            }
            if(balls.size() == 3 && blocking && wall_pose == wall_block){
                flippy_pos = flippy_up;
                if (stop_intake) {
                    intake.setPower(0);
                }else {
                    intake.setPower(.6);
                }
                spindexer.setPower(0);
            }else {
                if (stop_intake) {
                    intake.setPower(0);
                }else {
                    if(blocking) {
                        if(intake.getVelocity() < -500 && spit_count == 0){
                            intake_is_spinning = true;
                        }else if((intake.getVelocity() < -spit_speed && intake_is_spinning) || spit_count == 1){
                            spit_out();
                        }
                        spindexer.setPower(1);
                    }
                    if(spit_count == 0) {
                        intake.setPower(-1 + gamepad1.right_trigger);
                    }
                }
            }
            flippy.setPosition(flippy_pos);
            count();
        }
        public void count(){
            if(blocking) {
                if (balls.isEmpty()) {
                    if (disfirst.getState()) {
                        balls.add("Ball");
                        Light.setPower(.4);
                    }else{
                        Light.setPower(.5);
                    }
                } else if (balls.size() == 1) {
                    if (dismiddle.getState()) {
                        balls.add("Ball");
                        Light.setPower(.35);
                    }
                } else if (balls.size() == 2) {
                    if (dislast.getState()) {
                        balls.add("Ball");
                        Light.setPower(0);
                    }
                }
            }else{
                if(balls.size() - shooter.balls_shot == 2){
                    Light.setPower(.4);
                }else if(balls.size() - shooter.balls_shot == 1){
                    Light.setPower(.35);
                }else if(balls.size() - shooter.balls_shot == 0){
                    Light.setPower(0);
                }else{
                    Light.setPower(0.5);
                }
            }
        }


        public ElapsedTime spit = new ElapsedTime();
        public double spit_count = 0;
        public void spit_out(){
            if(spit_count == 0){
                spit.reset();
                spit_count = 1;
                intake.setPower(.6);
                intake_is_spinning = false;
            }else if(spit_count == 1 && spit.milliseconds() > spitcount && intake.getVelocity() > 0){
                spit_count = 0;
            }
        }


    }



    public class turret{
        public PIDController controller;

        public double p = .01, i = 0, d = .001;
        public double f = 0;
        public DcMotorEx turret;
        public double current_angle;
        public double limit = 180;
        public double turret_angle;
        public double power;
        public boolean zero = false;
        public boolean shootable = false;
        public double turret_target = 0;
        public double manual_angle = 0;
        public Servo turret_servo_1;
        public Servo turret_servo_2;
        public double turret_pos = 0;
        public double analog_offset = 0;
        public double servo_degrees = 90/.18;
        public double max = 25;
        public double multiplier = 1.2;
        public double a_slow = 1.2;
        public double angle_mod = .9;
        public double target_angle = .5;
        public double on_limit = 0;
        public AnalogInput servo_pose;
        public void initialize(){
            turret_servo_1 = hardwareMap.get(Servo.class, "turret_servo_1");
            turret_servo_2 = hardwareMap.get(Servo.class, "turret_servo_2");
            servo_pose = hardwareMap.get(AnalogInput.class, "servo_pose");
            turret_servo_1.setDirection(Servo.Direction.REVERSE);
            turret_servo_2.setDirection(Servo.Direction.REVERSE);
        }
        public void turret_move(){
            dead_wheel_calculations();
            double x = drive.position.getX(DistanceUnit.INCH);
            double y = drive.position.getY(DistanceUnit.INCH);
            if(zero){
                target_angle = manual_angle / servo_degrees + .5;
            }else{
                double angle = Math.toDegrees(location.getHeading());
                if (angle > 180) {
                    angle = angle - 360;
                }else if(angle < -180){
                    angle = 360 + angle;
                }else{
                    angle *= -1;
                }
                angle = dead_angle - angle;
                if (angle > 180) {
                    angle = angle - 360;
                }else if(angle < -180){
                    angle = 360 + angle;
                }
                target_angle = (angle) / servo_degrees + .5;
            }
            zero = false;
            if((target_angle - .5) * servo_degrees > limit){
                if(on_limit == 0) {
                    target_angle = (limit - 2) / servo_degrees + .5;
                }else{
                    target_angle = on_limit;
                }
                on_limit = target_angle;
            }else if((target_angle - .5) * servo_degrees < -limit){
                if(on_limit == 0) {
                    target_angle = (-limit + 2) / servo_degrees + .5;
                }else{
                    target_angle = on_limit;
                }
                on_limit = target_angle;
            }else{
                on_limit = 0;
            }
            if(manual_turret){
                target_angle = .5;
            }
            turret_servo_1.setPosition(target_angle + .015);
            turret_servo_2.setPosition(target_angle - .015);


        }
    }
    public double dead_angle = 0;
    public double dead_distance = 0;
    public static double y_mod = -4;
    public static double x_mod = -5;
    public Pose location = new Pose();
    public double calc_angle = 0;
    public double calc_distance = 0;
    public double last_x = 0;
    public double velocity_x = 0;
    public double acceleration_x = 0;
    public double next_x = 0;
    public double last_y = 0;
    public double velocity_y = 0;
    public double acceleration_y = 0;
    public double next_y = 0;
    public double last_heading = 0;
    public double velocity_heading = 0;
    public double acceleration_heading = 0;
    public double next_heading = 0;
    public void dead_wheel_calculations(){
        double x_changes = 0;
        if(color == 0){
            location = new Pose((130 + x_mod + x_changes) - (drive.position.getX(DistanceUnit.INCH) + 2 * cos(drive.position.getHeading(AngleUnit.RADIANS))), (52.2 + y_mod) - (drive.position.getY(DistanceUnit.INCH) + 2 * sin(drive.position.getHeading(AngleUnit.RADIANS))), drive.position.getHeading(AngleUnit.RADIANS));
        }else{
            location = new Pose((130 + x_mod + x_changes) - (drive.position.getX(DistanceUnit.INCH) + 2 * cos(drive.position.getHeading(AngleUnit.RADIANS))), (52.2 + y_mod) + (drive.position.getY(DistanceUnit.INCH) + 2 * sin(drive.position.getHeading(AngleUnit.RADIANS))), drive.position.getHeading(AngleUnit.RADIANS));
        }
        if(last_x == 0){
            last_x = location.getX();
            last_y = location.getX();
        }
        acceleration_x = (location.getX() - last_x) - velocity_x;
        velocity_x = location.getX() - last_x;
        next_x = location.getX() + (velocity_x + acceleration_x);
        acceleration_y = (location.getY() - last_y) - velocity_y;
        velocity_y = location.getY() - last_y;
        next_y = location.getY() + (velocity_y + acceleration_y);
        acceleration_heading = (location.getHeading() - last_heading) - velocity_heading;
        velocity_heading = location.getHeading() - last_heading;
        next_heading = location.getHeading() + (velocity_heading + acceleration_heading);
        last_x = location.getX();
        last_y = location.getY();
        last_heading = location.getHeading();
        location = new Pose(next_x, next_y, next_heading);
        double old_angle = calc_angle;
        double old_distance = calc_distance;
        if(location.getX() <= 0){
            dead_angle = 90;
        }else{
            dead_angle = Math.toDegrees(Math.atan(location.getY() / location.getX()));
        }
        if(color == 0){
            dead_angle *= -1;
        }
        dead_distance = Math.sqrt(Math.pow(location.getX(), 2) + Math.pow(location.getY(), 2));

    }
    public class webcam{
        public AprilTagProcessor aprilTag = null;
        public List<Double> tags = new ArrayList<>();

        /**
         * The variable to store our instance of the vision portal.
         */
        public VisionPortal visionPortal;
        public void initialize(){
            if(aprilTag == null) {
                aprilTag = AprilTagProcessor.easyCreateWithDefaults();
                visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), aprilTag);
            }
        }
        public void look(){
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            tags.clear();
            for (AprilTagDetection detections : currentDetections){
                tags.add((double) detections.id);
            }
        }
    }
    public class CameraCode{
        public Limelight3A limelight;
        public double x = 0;
        public double y = 0;
        public double r = 0;
        public double z = 0;
        public double ticks_per_degree = 5.228;
        public double tag_to_target_distance = .47;
        public double distance_from_target = 0;
        public double angle_from_target = 0;
        public double robot_x = 0;
        public double robot_y = 0;
        public double robot_angle;
        public boolean robot_auto_on = false;
        public double feet = .3048;
        public double angle = 0;
        public LLResult result;
        public double[] python = new double[0];
        public ElapsedTime waiter = new ElapsedTime();
        public void init() {
            limelight = hardwareMap.get(Limelight3A.class,"limelight");
            waiter.reset();
            limelight.start();
            if(color == 1){
                limelight.pipelineSwitch(1);
            }else {
                limelight.pipelineSwitch(0);
            }
        }

    }






}
