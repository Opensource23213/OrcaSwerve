package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import android.os.Build;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.BlueSwerveConst;
import org.firstinspires.ftc.teamcode.pedroPathing.SwerveConst;
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
    public static Pose auto_pose = new Pose();

    public double old_dis = 0;
    public intake intake_system = new intake();
    public shooter shooter = new shooter();
    public turret turret = new turret();
    public ElapsedTime flip_up = new ElapsedTime();
    public static double shoot_power_offset = 1220;
    public static double shoot_multiplier = 220;
    public boolean teleop = true;
    public double index_steps = 0;
    public List<LynxModule> hubs = null;
    public  webcam cam = new webcam();
    public static double pattern = 1;
    public static double anglex_mod = 60;
    public static double x_change = -6;
    @Override
    public void init(){
        initialize();
    }

    public void initialize() {
        x_mod = -5;
        y_mod = -5;
        if(color == 0){
            follower = BlueSwerveConst.createFollower(hardwareMap, gamepad1, color);
        }else {
            follower = SwerveConst.createFollower(hardwareMap, gamepad1, color);
        }
        shooter.initialize();
        turret.initialize();
        intake_system.init();
        cam.initialize();
        follower.update();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public void switch_constants(FollowerConstants constant){
        follower.constants.drivePIDFCoefficients(constant.getCoefficientsDrivePIDF());
        follower.constants.headingPIDFCoefficients(constant.getCoefficientsHeadingPIDF());
        follower.constants.translationalPIDFCoefficients(constant.getCoefficientsTranslationalPIDF());
    }

    @Override
    public void loop() {
    }
    public Button1 button1 = new Button1();
    public class Button1{
        List<String> button = new ArrayList<>();
        List<String> nowbutton = new ArrayList<>();
        List<String> lastbutton = new ArrayList<>();
        String type = "";
        public void button(){
            ButtonControl();
        }
        public void ButtonControl(){

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
                x_mod += 1;
            }else if(gamepad1.dpadDownWasReleased() || gamepad2.dpadDownWasReleased()){
                x_mod -= 1;
            }
            if(gamepad1.a){
                x_mod += 123 - location.getX();
            }else if(gamepad1.b){
                y_mod += 130 - location.getY();
            }else if(gamepad1.ps){
                x_mod += 123 - location.getX();
                y_mod += 130 - location.getY();
            }

        }

    }


    public boolean robot_going_forward = true;
    public Follower follower;

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
                telemetry.addData("Balls Shot", shooter.total_balls_shot);
                turret.turret_move();
                intake_system.tele();
                button1.button();
                if (shooter.balls_shot < intake_system.balls.size() && gamepad1.left_trigger > .4) {
                    if ((dead_distance * .0254) > 3) {
                        follower.drivetrain.setXVelocity(0);
                    } else {
                        follower.drivetrain.setYVelocity(0);
                    }

                }
                if (gamepad2.touchpad) {
                    tippy_toe = true;
                }
                follower.update();
            } else if (!gamepad1.atRest()) {
                start = true;
            }
        }
    }
    public static double alt_flap = .1;
    public static double speed_increase = 220;
    public static double flap_distance = 2;
    public class shooter{
        public Servo flap;
        public DcMotorEx shoot1;
        public DcMotorEx shoot2;
        public double speed = 0;
        public double position = 0.05;
        public double speed_difference = 0;
        public double max_draw1 = 0;
        public double max_draw2 = 0;
        public double calc_velocity = 0;
        public double calc_rpm = 0;
        public double flap_velocity = 0;
        public double old_velocity = 0;
        public double last_shot = 1600;
        public boolean far_shooting = false;
        public double set_speed = 0;
        public void initialize(){
            flap = hardwareMap.get(Servo.class, "flap");
            PIDFCoefficients pidf = new PIDFCoefficients(100, .5, .001, 7);
            shoot2 = hardwareMap.get(DcMotorEx.class, "shoot2");
            shoot1 = hardwareMap.get(DcMotorEx.class, "shoot1");
            shoot2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
            shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void shooting() {
            flap_mod();
            shoot2.setVelocity(speed);
            shoot1.setVelocity(speed);
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
            if(teleop) {
                shooter.set_speed = shoot_multiplier * ((dead_distance * .0254) - 1.6) + shoot_power_offset + (speed_increase * abs(turret.target_angle - .5));
                shooter.set_speed = Math.round(shooter.set_speed / 20) * 20;
            }
            if(!intake_system.blocking) {
                if (first_press == 0) {
                    start_speed = shoot1.getVelocity();
                    first_press = 1;
                    least_speed = start_speed;
                    speed = set_speed;
                } else if (first_press == 1) {
                    if (shoot1.getVelocity() < least_speed) {
                        least_speed = shoot1.getVelocity();
                    } else if (shoot1.getVelocity() > least_speed + 60) {
                        first_press = 2;
                        balls_shot += 1;
                        start_speed = shoot1.getVelocity();
                        least_speed = start_speed;
                    }
                    speed = set_speed;

                } else if (first_press == 2) {
                    speed = set_speed;
                    if (shoot1.getVelocity() > start_speed) {
                        start_speed = shoot1.getVelocity();
                        least_speed = shoot1.getVelocity();
                    } else if (shoot1.getVelocity() < least_speed) {
                        least_speed = shoot1.getVelocity();
                        first_press = 3;
                    }
                } else if (first_press == 3) {
                    speed = set_speed;
                    if (shoot1.getVelocity() < least_speed) {
                        least_speed = shoot1.getVelocity();
                    } else if (shoot1.getVelocity() > least_speed + 60) {
                        first_press = 2;
                        balls_shot += 1;
                        start_speed = shoot1.getVelocity();
                        least_speed = start_speed;
                    }
                }
                telemetry.addData("Balls Shot", balls_shot);
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
    public static double flippy_up = 0;
    public static double flippy_hold = .45;
    public static double  flippy_down = .45;
    public static double mod = 200;
    public static double dis = .8;
    public static double speed_int = -1;
    public static double shot_speed = .35;
    public static double wall_block = .575;
    public static double color_distance = 80;
    public static double sorting_speed = 200;
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
        public Servo wall;
        public color_sensor color_first = new color_sensor();
        public color_sensor color_middle = new color_sensor();
        public color_sensor color_last = new color_sensor();
        public List<String> balls = new ArrayList<>();
        public AnalogInput spin_dis;
        public boolean outtake = false;
        public boolean shot_right = false;
        public double wall_clear = .25;
        public double wall_pose = wall_block;
        public boolean was_shooting = false;
        public boolean shoot_slower = false;
        public CRServo Light;
        public void init(){
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
            spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flippy = hardwareMap.get(Servo.class, "flippy");
            blocker = hardwareMap.get(Servo.class, "blocker");
            Light = hardwareMap.get(CRServo.class, "light");
            wall = hardwareMap.get(Servo.class, "wall");
            color_first.init("color_first1", "color_first2");
            color_middle.init("color_middle1", "color_middle2");
            color_last.init("color_last1", "color_last2");
            Light.setDirection(DcMotorSimple.Direction.REVERSE);
            spin_dis = hardwareMap.get(AnalogInput.class, "spin_dis");
        }
        public void tele(){
            auto_sort();
            sort();
            if(gamepad1.left_trigger > .4){
                sorting = false;
                wall_pose = wall_block;
                sort_balls = false;
                if((dead_distance * .0254) > 3){
                    spindexer.setPower(-.6);
                }else {
                    if(shoot_slower){
                        spindexer.setPower(-.7);
                    }else{
                        spindexer.setPower(-1);
                    }

                }
                flippy_pos = flippy_up;
                blocking = false;
            }else{
                if(gamepad2.right_bumper){
                    gamepad1.rumble(1000);
                    shoot_slower = true;
                    sort_balls = true;
                }
                flippy_pos = flippy_down;
                blocking = true;
            }
            if(balls.size() == 3 && blocking && wall_pose == wall_block){
                if(sorting){
                    if(sort_time == 0){
                        sorting = false;
                    }
                }else {
                    if (stop_intake) {
                        intake.setPower(0);
                    }else {
                        intake.setPower(.6);
                    }
                    spindexer.setPower(0);
                }
                spindex_outtake = true;
            }else {
                if(blocking && !sorting) {
                    spindexer.setPower(1);
                }
                intake.setPower(-1 + gamepad1.right_trigger);
            }
            flippy.setPosition(flippy_pos);
            if(blocking){
                blocker.setPosition(block);
            }else{
                blocker.setPosition(unblock);
                spindex_outtake = false;
            }
            wall.setPosition(wall_pose);
            count();
            telemetry.addData("Color of Balls", balls);
        }
        public boolean stop_intake = false;
        public void auto(){
            if(blocking){
                blocker.setPosition(block);
            }else{
                blocker.setPosition(unblock);
            }
            if(balls.size() == 3 && blocking && wall_pose == wall_block){
                if(sorting){
                    if(sort_time == 0){
                        sorting = false;
                    }
                }else {
                    if (stop_intake) {
                        intake.setPower(0);
                    }else {
                        intake.setPower(1);
                    }
                    spindexer.setPower(0);
                }
                spindex_outtake = true;
            }else {
                if (stop_intake) {
                    intake.setPower(0);
                }else {
                    intake.setPower(-1);
                }
            }
            flippy.setPosition(flippy_pos);
            wall.setPosition(wall_pose);
            count();
            telemetry.addData("Color of Balls", balls);
        }
        public void count(){
            if(blocking) {
                if (balls.isEmpty()) {
                    if (color_first.is_ball()) {
                        balls.add(color_first.color());
                        Light.setPower(.4);
                    }else{
                        Light.setPower(.5);
                    }
                } else if (balls.size() == 1) {
                    if (color_middle.is_ball()) {
                        balls.add(color_middle.color());
                        Light.setPower(.35);
                    }
                } else if (balls.size() == 2) {
                    if (color_last.is_ball()) {
                        balls.add(color_last.color());
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
        public void recount(){
            balls.clear();
            balls.add(color_first.color());
            balls.add(color_middle.color());
            balls.add(color_last.color());
        }
        public class color_sensor{
            public RevColorSensorV3 color1;
            public RevColorSensorV3 color2;
            public void init(String sensor1, String sensor2){
                color1 = hardwareMap.get(RevColorSensorV3.class, sensor1);
                color1.enableLed(false);
                color2 = hardwareMap.get(RevColorSensorV3.class, sensor2);
                color2.enableLed(false);
            }
            public String color(){
                if(is_ball()) {
                    if (color1.getDistance(DistanceUnit.MM) < color2.getDistance(DistanceUnit.MM)) {
                        if (color1.getNormalizedColors().green > color1.getNormalizedColors().blue) {
                            return "Green";
                        } else {
                            return "Purple";
                        }
                    } else {
                        if (color2.getNormalizedColors().green > color2.getNormalizedColors().blue) {
                            return "Green";
                        } else {
                            return "Purple";
                        }
                    }
                }else{
                    return "No Ball";
                }
            }
            public List<Integer> getColors(){
                if (color1.getDistance(DistanceUnit.MM) < color2.getDistance(DistanceUnit.MM)) {
                    return List.of(color1.alpha(), color1.red(), color1.green(), color1.blue());
                } else {
                    return List.of(color2.alpha(), color2.red(), color2.green(), color2.blue());
                }
            }
            public boolean is_ball(){
                if(color2.getDistance(DistanceUnit.MM) < color_distance || color1.getDistance(DistanceUnit.MM) < color_distance){
                    return true;
                }else{
                    return false;
                }

            }
        }
        public boolean spindex_outtake = false;
        public void intake_test(){
            if(gamepad1.left_trigger > .4){
                flippy_pos = flippy_up;
                blocking = false;
            }else{
                if(gamepad1.a){
                    spindexer.setPower(1);
                    flippy_pos = flippy_down;
                    blocking = true;
                }else if(gamepad1.b) {
                    spindexer.setPower(speed_int);
                    flippy_pos = flippy_up;
                    blocking = false;
                }else if(gamepad1.y){
                    sort_time = 1;
                }else if(gamepad1.x){
                    sort_time = 2;
                }else if(gamepad1.right_bumper){
                    sort_balls = true;
                    override = true;
                }else{
                    blocking = true;
                }
            }
            if(balls.size() == 3 && blocking && wall_pose == wall_block){
                if(sorting && sort_time == 0){
                    if(color_last.is_ball()){
                        sorting = false;
                    }
                }else {
                    intake.setPower(1);
                    spindexer.setPower(0);
                }
                spindex_outtake = true;
            }else {
                intake.setPower(-1 + gamepad1.right_trigger);
            }
            auto_sort();
            sort();
            flippy.setPosition(flippy_pos);
            if(blocking){
                if(was_shooting){
                    spindexer.setPower(0);
                    flippy_pos = flippy_hold;
                    was_shooting = false;
                }
                blocker.setPosition(block);
            }else{
                blocker.setPosition(unblock);
                spindex_outtake = false;
                was_shooting = true;
            }
            wall.setPosition(wall_pose);
            count();
            telemetry.addData("Color of Balls", balls);
        }
        public ElapsedTime flip = new ElapsedTime();
        public boolean was_closed = true;

        public boolean sorting = false;
        public double sort_time = 0;
        public void sort(){
            if(sort_time > 0){
                blocking = true;
                flippy_pos = flippy_down;
                if(!sorting){
                    flip.reset();
                }
                if(sort_time == 1){
                    if (flip.milliseconds() < sorting_speed) {
                        spindexer.setPower(.6);
                        wall_pose = wall_clear;
                        sorting = true;
                    } else {
                        wall_pose = wall_block;
                        sort_time = 0;
                    }
                }else{
                    if (flip.milliseconds() < sorting_speed) {
                        spindexer.setPower(.6);
                        wall_pose = wall_clear;
                        sorting = true;
                    } else if(flip.milliseconds() < sorting_speed + 200){
                        wall_pose = wall_block;
                    }else if(flip.milliseconds() < sorting_speed + 200 + sorting_speed){
                        spindexer.setPower(.6);
                        wall_pose = wall_clear;
                    }else{
                        wall_pose = wall_block;
                        sort_time = 0;
                    }
                }
            }
        }
        public double sort_twice = 0;
        public void sort_twice(){
            if(sort_twice == 1){
                sort_time = 1;
                sort_twice = 2;
            }else if(!sorting && sort_twice == 2){
                sort_time = 1;
                sort_twice = 0;
            }
        }
        public int sort_count(){
            intake_system.recount();
            List<String> unsort = new ArrayList<>();
            if(intake_system.balls.size() == 1){
                unsort.add(intake_system.balls.get(0));
            }else if(intake_system.balls.size() == 2){
                unsort.add(intake_system.balls.get(1));
                unsort.add(intake_system.balls.get(0));
            }else if(intake_system.balls.size() == 3){
                unsort.add(intake_system.balls.get(2));
                unsort.add(intake_system.balls.get(1));
                unsort.add(intake_system.balls.get(0));
            }
            List<String> sort = new ArrayList<>();
            int num = 0;
            if(pattern == 1){
                sort.add("Green");
                sort.add("Purple");
                sort.add("Purple");
            }else if(pattern == 2){
                sort.add("Purple");
                sort.add("Green");
                sort.add("Purple");
            }else if(pattern == 3){
                sort.add("Purple");
                sort.add("Purple");
                sort.add("Green");
            }
            if(unsort.size() == 3) {
                if (unsort == sort) {
                    num = 0;
                } else if (!unsort.contains("Green")) {
                    num = 0;
                } else if (!unsort.contains("Purple")) {
                    num = 0;
                } else {
                    if (Collections.frequency(unsort, "Green") > 1) {
                        num = sort.indexOf("Purple") - unsort.indexOf("Purple");
                        if (num < 0) {
                            num += 3;
                        }
                    } else {
                        num = sort.indexOf("Green") - unsort.indexOf("Green");
                        if (num < 0) {
                            num += 3;
                        }
                    }
                }
            }else{
                if(unsort.contains("Green")){
                    if(sort.indexOf("Green") != unsort.indexOf("Green")){
                        num = 1;
                    }else {
                        num = 0;
                    }
                }else{
                    num = 0;
                }
            }
            return num;
        }
        public boolean sorted = false;
        public double sort_steps = 0;
        public ElapsedTime sort_timer = new ElapsedTime();
        public boolean sort_balls = false;
        public boolean override = false;
        public void auto_sort(){
            if(!sorted && intake_system.blocking && sort_balls) {
                if (intake_system.balls.size() == 3 || override) {
                    if(sort_steps == 0){
                        sort_timer.reset();
                        sort_steps = 1;
                    }else if(sort_steps == 1 && sort_timer.milliseconds() > 500){
                        sorted = true;
                        override = false;
                        intake_system.sort_time = sort_count();
                        sort_balls = false;
                        sort_steps = 0;
                    }
                }

            }else if(!intake_system.blocking){
                sorted = false;
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
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            turret_angle = (1.632 - servo_pose.getVoltage()) / .517 * 90;
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
            turret_servo_1.setPosition(target_angle + .0072);
            turret_servo_2.setPosition(target_angle - .0072);
            telemetry.addData("angle", turret_angle);


        }
    }
    public double dead_angle = 0;
    public double dead_distance = 0;
    public static double y_mod = -12;
    public static double x_mod = 2;
    public Pose location = new Pose();
    public double calc_angle = 0;
    public double calc_distance = 0;
    public void dead_wheel_calculations(){
        double x_changes = 0;
        if(teleop && color == 0) {
            if (abs(dead_angle) > anglex_mod) {
                x_changes = x_change;
            }
        }
        if(color == 0){
            location = new Pose((130 + x_mod + x_changes) - (follower.getPose().getX() + 2 * cos(follower.getPose().getHeading())), (52.2 + y_mod) - (follower.getPose().getY() + 2 * sin(follower.getPose().getHeading())), follower.getPose().getHeading());

        }else{
            location = new Pose((130 + x_mod + x_changes) - (follower.getPose().getX() + 2 * cos(follower.getPose().getHeading())), (52.2 + y_mod) + (follower.getPose().getY() + 2 * sin(follower.getPose().getHeading())), follower.getPose().getHeading());

        }
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
        telemetry.addData("distance", dead_distance * .0254);
        telemetry.addData("x", location.getX());
        telemetry.addData("y", location.getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("target_angle", (turret.target_angle - .5) * turret.servo_degrees);

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






}
