package frc.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PWM extends SubsystemBase {
    
    private static PWMVictorSPX spx_feed, spx_pickup, spx_turret;
    private static TalonFX fx_flybottom, fx_flytop;
    public boolean state_feed = false, state_launch = false, state_pickup = false;
    public boolean state_arm = false;
    private double speed_launch = 0.75;
    private final double kMaxTurretSpeed = 0.5;

    private Timer turretTimer;
    private final double kTurretLimit = 1.0;
    private double turretTime = 0.0;
    private boolean turretActive = false;

    public PWM() {
        fx_flybottom = new TalonFX(0);
        fx_flytop = new TalonFX(1);
        spx_feed = new PWMVictorSPX(Constants.PWM_FEED);
        spx_pickup = new PWMVictorSPX(Constants.PWM_PICKUP);
        spx_turret = new PWMVictorSPX(Constants.PWM_TURRET);
        SmartDashboard.putBoolean("Arm Active", false);
        SmartDashboard.putNumber("Launch Speed", 0.75);
    }

    public void setFeed(boolean b) {
        double speed_left = b ? 0.2 : 0.0;
        spx_feed.set(speed_left);
        state_feed = b;
    }

    public void setLaunch(String s) {
        if (s.equalsIgnoreCase("unstuck")) {
            fx_flybottom.set(ControlMode.PercentOutput, -0.3);
            fx_flytop.set(ControlMode.PercentOutput, 0.3);
            spx_feed.set(-0.2);
            Timer timer = new Timer();
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    fx_flybottom.set(ControlMode.PercentOutput, 0.0);
                    fx_flytop.set(ControlMode.PercentOutput, 0.0);
                    spx_feed.set(0.0);
                }
            }, 500);
        } else {
            double speed = (s.equalsIgnoreCase("go")) ? speed_launch : 0.0;
            fx_flybottom.set(ControlMode.PercentOutput, speed);
            fx_flytop.set(ControlMode.PercentOutput, -speed);
            state_launch = s.equalsIgnoreCase("go");
        }
    }

    public void setPickup(boolean b) {
        double speed = b ? 0.5 : 0.0;
        spx_pickup.set(speed);
        state_pickup = b;
    }
    
    @Override
    public void periodic() {
        double launchAxis = RobotContainer.joystick_1.getRawAxis(Constants.CTRL1_DIAL);
        speed_launch = (-launchAxis + 1.0) * 0.25 + 0.5;

        // Timing limiter for turret movement
        // Counter-clockwise
        if (RobotContainer.joystick_0.getRawButtonPressed(Constants.CTRL0_LB) && !turretActive && turretTime > -kTurretLimit) {
            turretActive = true;
            spx_turret.set(-1.0 * kMaxTurretSpeed);
            turretTimer = new Timer();
            turretTimer.schedule(new TimerTask() {
                @Override
                public void run() {
                    turretTime -= 0.1;
                    if (turretTime <= -kTurretLimit) {
                        spx_turret.set(0.0);
                        turretActive = false;
                        this.cancel();
                    }
                }
            }, 100, 100);
        }

        // Clockwise
        else if (RobotContainer.joystick_0.getRawButtonPressed(Constants.CTRL0_RB) && !turretActive && turretTime < kTurretLimit) {
            turretActive = true;
            spx_turret.set(1.0 * kMaxTurretSpeed);
            turretTimer = new Timer();
            turretTimer.schedule(new TimerTask() {
                @Override
                public void run() {
                    turretTime += 0.1;
                    if (turretTime >= kTurretLimit) {
                        spx_turret.set(0.0);
                        turretActive = false;
                        this.cancel();
                    }
                }
            }, 100, 100);
        }

        // Buttons released
        else if (!RobotContainer.joystick_0.getRawButton(Constants.CTRL0_LB) && !RobotContainer.joystick_0.getRawButton(Constants.CTRL0_RB) && turretActive) {
            turretActive = false;
            spx_turret.set(0.0);
            turretTimer.cancel();
        }

        SmartDashboard.putNumber("Launch Speed", speed_launch);
        SmartDashboard.putBoolean("Feed", state_feed);
        SmartDashboard.putBoolean("Launch", state_launch);
        SmartDashboard.putBoolean("Pickup", state_pickup);
    }
}
