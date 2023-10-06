package frc.ApriltagsCamera;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.ApriltagsCamera.Network.NetworkReceiver;
import frc.pathfinder.Pathfinder;
import frc.pathfinder.Pathfinder.Path;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Constants;
import frc.robot.ParadoxField;
import frc.robot.PositionTracker;

public class PositionServer implements NetworkReceiver {
    private Network m_network = new Network();
    private Timer m_watchdogTimer = new Timer();
    private boolean m_connected = false;
    private double m_xPos = 0;
    private double m_yPos = 0;
    private boolean m_newPos = true;
    private Object m_lock = new Object();
    private Object m_recvLock = new Object();
    // private Gyro m_gyro;

    double m_testX = 10 * 12;
    double m_testY = 0;
    double m_angle = 90;

    public void start() {
        // m_gyro = gyro;
        m_network.listen(this, 5802);

        m_watchdogTimer.scheduleAtFixedRate(new TimerTask() {

            @Override
            public void run() {
                // Logger.log("PositionServer", 1, "connected=" + m_connected);
                if (m_connected) {
                    Logger.log("PositionServer", -1, "Send position");

                    {

                        // m_network.sendMessage(String.format("+%.2f %.2f %.2f\n", m_angle, m_testX,
                        // m_testY));

                        // m_testX += 1;
                        // m_testY += 0.5;
                        // m_angle += 1.0/5;
                    }

                    double xPos;
                    double yPos;
                    double angle;
                    boolean newPos;

                    synchronized (m_lock) {
                        xPos = m_xPos;
                        yPos = m_yPos;
                        angle = m_angle;
                        newPos = m_newPos;

                        m_newPos = false;
                    }

                    if (newPos) {
                        m_network.sendMessage(String.format("+%.2f %.2f %.2f\n", angle, xPos, yPos));
                    } else {
                        m_network.sendMessage("-\n"); // keep alive
                    }

                    // if (m_lastMessage + k_timeout < System.currentTimeMillis()) {
                    // Logger.log("ApriltagsCamera", 3, "Network timeout");
                    // m_network.closeConnection();
                    // }
                }
            }
        }, 200, 200); // Send current position 5 times a second
    }

    public void setPosition(double x, double y, double angle) {
        synchronized (m_lock) {
            m_xPos = x * 12;
            m_yPos = y * 12;
            m_angle = angle;
            m_newPos = true;
        }
    }

    boolean m_redAlliance;

    public void setAllianceColor(boolean red) {
        ApriltagLocations.setColor(!red);

        if (red != m_redAlliance) {
            m_redAlliance = red;

            if (m_connected) {
                sendAllianceColor();
            }
        }
    }

    public class BezierData {
        public double m_x1; // In feet
        public double m_y1; // In feet
        public double m_angle1; // In radians
        public double m_l1; // In feet
        public double m_x2; // In feet
        public double m_y2; // In feet
        public double m_angle2; // In radians
        public double m_l2; // In Feet

        public BezierData(double x1, double y1, double angle1, double l1, double x2, double y2, double angle2,
                double l2) {
            Logger.log("BezierData", 1,
                    String.format("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", x1, y1, angle1, l1, x2, y2, angle2, l2));

            m_x1 = x1;
            m_y1 = y1;
            m_angle1 = angle1;
            m_l1 = l1;
            m_x2 = x2;
            m_y2 = y2;
            m_angle2 = angle2;
            m_l2 = l2;
        }
    }

    private BezierData[] m_data;
    private BezierData[] m_newData;
    private int m_nBezier;

    private void processPath(String path) {
        Logger.log("PositionServer", -1, String.format("processPath: %s", path));

        int[] args = ApriltagsCamera.parseIntegers(path, 1);

        if (args != null) {
            m_nBezier = 0;
            m_newData = new BezierData[args[0]];
        }

    }

    private void processBezier(String bezier) {
        Logger.log("PositionServer", 1, String.format("processBezier: cnt=%d %s", m_nBezier, bezier));

        if ((m_newData != null) && (m_nBezier < m_newData.length)) {
            double[] arg = ApriltagsCamera.parseDouble(bezier, 8);
            m_newData[m_nBezier] = new BezierData(arg[0], arg[1], arg[2], arg[3], arg[4], arg[5], arg[6], arg[7]);
        } else {
            m_newData = null;
            Logger.log("PositionServer", 3, String.format("Invalid Bezier Command: %s", bezier));
        }

        m_nBezier++;
    }

    private void processEnd(String end) {
        if (m_newData != null) {
            if (m_nBezier == m_newData.length) {
                synchronized (m_recvLock) {
                    m_data = m_newData;
                    m_newData = null;
                }
            } else {
                Logger.log("PositionServer", 3,
                        String.format("Invalid Bezier Length: %d/%d", m_nBezier, m_newData.length));
            }
        }
    }

    public class Target {
        public int m_no;
        public int m_level;
        public boolean m_isCone;
        public double m_x;
        public double m_y;
        public double m_h;
        public double m_ext; // extent adjustment

        public double m_endpointX0;
        public double m_endpointLeftX;
        public double m_endpointLeftAngle;
        public double m_endpointRightX;
        public double m_endpointRightAngle;
        public double m_endpointP1;
        public double m_endpointY;
        public double m_endpointP0;

        Target(int no, int level, double x, double y, double h, double type, double ext,
                double x0, double leftX, double leftAngle, double rightX, double rightAngle, double p1,
                double endpointY, double p0) {
            m_no = no;
            m_level = level;
            m_x = x;
            m_y = y;
            m_h = h;
            m_ext = ext;
            m_isCone = type == 1;

            SmartDashboard.putNumber("Target no", no);
            SmartDashboard.putNumber("Target level", level);

            m_endpointX0 = x0;
            m_endpointLeftX = leftX;
            m_endpointLeftAngle = leftAngle;
            m_endpointRightX = rightX;
            m_endpointRightAngle = rightAngle;
            m_endpointP1 = p1;
            m_endpointY = endpointY;
            m_endpointP0 = p0;

            SmartDashboard.putNumber("Path x0", x0);
            SmartDashboard.putNumber("Path leftX", leftX);
            SmartDashboard.putNumber("Path leftAngle", leftAngle);
            SmartDashboard.putNumber("Path rightX", rightX);
            SmartDashboard.putNumber("Path rightAngle", rightAngle);
            SmartDashboard.putNumber("Path p1", p1);
            SmartDashboard.putNumber("Path y", endpointY);
            SmartDashboard.putNumber("Path p0", p0);
        }

        private static final int k_nPoints = 1000;
        private static final double k_dt = 0.02;
        private static final double k_wheelbase = Constants.Drive.k_wheelBase;
        private static final double k_maxSpeed = 4;
        private static final double k_maxAccel = 5.000000;
        private static final double k_maxDecl = 5.000000;
        private static final double k_maxJerk = 100;

        public Path getPath(PositionTracker tracker) {
            Pose2d pos = tracker.getPose2d();
            double robotX = pos.getX();
            double robotY = pos.getY();
            double robotAngle = pos.getRotation().getDegrees();

	    	double endX;
	    	double endAngle;
	    	
	    	if ((robotX < m_endpointX0) ^ !m_redAlliance)
	    	{
	    		// to right
		    	endX = m_endpointRightX;
		    	endAngle = Math.toRadians(m_endpointRightAngle);
	    	}
	    	else
	    	{
	    		// to left
		    	endX = m_endpointLeftX;
		    	endAngle = Math.toRadians(m_endpointLeftAngle);	    		
	    	}

            double dx = endX - robotX;
            double dy = m_endpointY - robotY;
            double d = Math.sqrt(dx*dx + dy*dy);
	    	
            Waypoint[] waypoints = {
                    new Waypoint(robotX, robotY, Math.toRadians(robotAngle), m_endpointP0 * d, m_endpointP1 * d),
                    new Waypoint(endX, m_endpointY, endAngle)
            };

            SmartDashboard.putNumber("Waypoint1 x", waypoints[0].x);
            SmartDashboard.putNumber("Waypoint1 y", waypoints[0].y);
            SmartDashboard.putNumber("Waypoint1 angle", Math.toDegrees(waypoints[0].angle));
            SmartDashboard.putNumber("Waypoint1 l1", waypoints[0].l1);
            SmartDashboard.putNumber("Waypoint1 l2", waypoints[0].l2);

            SmartDashboard.putNumber("Waypoint2 x", waypoints[1].x);
            SmartDashboard.putNumber("Waypoint2 y", waypoints[1].y);
            SmartDashboard.putNumber("Waypoints2 angle", Math.toDegrees(waypoints[1].angle));

            return Pathfinder.computePath(waypoints, k_nPoints, k_dt, k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk,
                    k_wheelbase);
        }

        // Returns true if the robot must run backwards
        public boolean isPathReversed(PositionTracker tracker) {
            Pose2d pos = tracker.getPose2d();
            double dx = m_x - pos.getX();
            double dy = m_y - pos.getY();
            double a = Math.toDegrees(Math.atan2(dy, dx));                  // Angle to target
            double yaw = tracker.getPose2d().getRotation().getDegrees();
            double da = ParadoxField.normalizeAngle(yaw - a);    // Difference between the robot heading and the target

	    	return ((da > 90) || (da < -90));   // If more than 90 degrees off, run robot backwards
        }
    }

    private Target m_target = null;

    private void processTarget(String target) {
        double[] arg = ApriltagsCamera.parseDouble(target, 15);

        synchronized (m_lock) {
            m_target = new Target((int) arg[0], (int) arg[1], arg[2], arg[3], arg[4], arg[5], arg[6],
                    arg[7], arg[8], arg[9], arg[10], arg[11], arg[12], arg[13], arg[14]);
        }
    }

    public Target getTarget() {
        synchronized (m_lock) {
            return m_target;
        }
    }

    @Override
    public void processData(String data) {
        Logger.log("PositionServer", 1, String.format("Data: %s", data));

        switch (data.charAt(0)) {
            case 'P': // path
                processPath(data.substring(1).trim());
                break;

            case 'B': // Bezier curve
                processBezier(data.substring(1).trim());
                break;

            case 'E': // End
                processEnd(data.substring(1).trim());
                break;

            case 'T': // Target Info
                processTarget(data.substring(1).trim());
                break;

            case 'k': // keep alive
                break;

            default:
                Logger.log("PositionServer", 3, String.format("Invalid command: %s", data));
                break;
        }
    }

    public BezierData[] getData() {
        BezierData[] data;

        synchronized (m_recvLock) {
            data = m_data;
            m_data = null;
        }

        return (data);
    }

    private void sendAllianceColor() {
        Logger.log("PositionServer", 1, String.format("sendAllianceColor: red =%b", m_redAlliance));
        m_network.sendMessage(String.format("c%c", m_redAlliance ? 'r' : 'b'));
    }

    @Override
    public void connected() {
        Logger.log("PositionServer", 1, String.format("connected: red=%b", m_redAlliance));

        m_connected = true;

        sendAllianceColor();
    }

    @Override
    public void disconnected() {
        Logger.log("PositionServer", 1, "disconnected");

        m_connected = false;
    }

}
