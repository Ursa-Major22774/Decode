package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.resources.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Red Auto", group = "Competition Autos")
@Configurable
public class RedAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opmodeTimer;
    private int pathState = 1; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Shooter shooter;
    private Turret turret;
    private Transfer transfer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shooter = new Shooter(hardwareMap, follower);
        turret = new Turret(hardwareMap, follower);
        transfer = new Transfer(hardwareMap);

        if (Utilities.getBatteryVoltage(hardwareMap) > 13.0) {
            shooter.adjustFlywheelSpeed(-0.2);
        } else if (Utilities.getBatteryVoltage(hardwareMap) > 12.5) {
            shooter.adjustFlywheelSpeed(-0.1);
        } else {
            shooter.adjustFlywheelSpeed(0);
        }
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Configurable
    public static class Paths {

        public static PathChain Path1;
        public static PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 8.000),
                                    new Pose(88.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 88.000),
                                    new Pose(110.000, 71.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch(pathState) {
            case 1:
                follower.followPath(paths.Path1);
                setPathState(pathState + 1);

                turret.reset();
                shooter.idle();

                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 6) {
                    turret.aim(false);
                    shooter.accelerateFlywheel(false);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 5 && pathTimer.getElapsedTimeSeconds() < 6) {
                    transfer.kick();
                }
                if ((!follower.isBusy()) && pathTimer.getElapsedTimeSeconds() > 7){
                    follower.followPath(paths.Path2);
                    setPathState(pathState + 1);
                    turret.reset();
                    shooter.idle();
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    pathState = 0;
                    transfer.resetKick();
                }
                break;
        }

        return pathState;
    }

    public void setPathState (int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}
