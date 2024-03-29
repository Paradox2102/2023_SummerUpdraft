/*
 *	  Copyright (C) 2022  John H. Gaby
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, version 3 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *    
 *    Contact: robotics@gabysoft.com
 */

package frc.ApriltagsCamera;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ParadoxField;

/**
 * 
 * @author John Gaby
 * 
 * @brief The ApriltagsCamera class handles communication with the Raspberry Pi
 *        Camera.
 * 
 *        This class allows you to connect to the Data server of the Raspberry
 *        Pi and receive information about the image frames processed by the
 *        camera.
 *
 *        It also implements a time sync protocol to synchronize the clocks and
 *        performs keep-alive functions that detect a disconnect.
 *
 */
public class ApriltagsCamera implements frc.ApriltagsCamera.Network.NetworkReceiver {
	/**
	 * @brief The ApriltagsCameraStats class collects the current camera performance
	 *        statistics.
	 *
	 */
	public class ApriltagsCameraStats {
		public int m_averageDelay = 0; // !<Specifies the average frame delay
		public int m_maxDelay = 0; // !<Specifies the max frame delay
		public int m_minDelay = Integer.MAX_VALUE; // !<Specifies the min frame delay
		public int m_lostFrames; // !<Specifies the number of lost frames
		public long m_time; // !<Specifies the elapsed time

		public ApriltagsCameraStats(int avgDelay, int maxDelay, int minDelay, int lostFrames, long time) {
			m_averageDelay = avgDelay;
			m_maxDelay = maxDelay;
			m_minDelay = minDelay;
			m_lostFrames = lostFrames;
			m_time = time;
		}
	}

	/**
	 * @brief The ApriltagsCameraRegion specifies a single detected region
	 *
	 */
	public class ApriltagsCameraRegion {
		public int m_tag; // !<Specifies the region color [0..3]
		public double m_rvec[] = new double[3];
		public double m_tvec[] = new double[3];
		public double m_corners[][] = new double[4][2];
		public double m_relAngleInDegrees;
		public double m_angleInDegrees;
		public double m_angleOffset;

		// ! @cond PRIVATE
		public ApriltagsCameraRegion(int tag, double r0, double r1, double r2, double t0, double t1, double t2,
				double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
			m_tag = tag;
			m_rvec[0] = r0;
			m_rvec[1] = r1;
			m_rvec[2] = r2;
			m_tvec[0] = t0;
			m_tvec[1] = t1;
			m_tvec[2] = t2;
			m_corners[0][0] = x0;
			m_corners[0][1] = y0;
			m_corners[1][0] = x1;
			m_corners[1][1] = y1;
			m_corners[2][0] = x2;
			m_corners[2][1] = y2;
			m_corners[3][0] = x3;
			m_corners[3][1] = y3;

			m_angleInDegrees = 
			m_relAngleInDegrees = r1 * 52; // + m_cameraAngleDegrees;
			// m_angleOffset = m_angleOffsetInDegrees;
		}
		// ! @endcond

				/*
		 * Finds the location of the tag with the specified ID
		 */
		// ApriltagLocation findTag(ApriltagLocation[] tags, int tagId) {
		// 	for (int i = 0; i < tags.length; i++) {
		// 		if (tags[i].m_tag == tagId) {
		// 			return (tags[i]);
		// 		}
		// 	}

		// 	return (null);
		// }

		// class Point
		// {
		// 	double x;
		// 	double y;

		// 	Point(double x, double y)
		// 	{
		// 		this.x = x;
		// 		this.y = y;
		// 	}
		// }

		Pose2d translatePos2d(double xPos, double yPos, double angleInRadians)
		{
			double a = Math.atan2(m_yOffsetInches, m_xOffsetInches);
			double d = Math.sqrt(m_xOffsetInches*m_xOffsetInches + m_yOffsetInches*m_yOffsetInches);
			double b = Math.PI/2 - angleInRadians - a;
			double dx = d * Math.cos(b);
			double dy = d * Math.sin(b);

			// Logger.log("ApriltagsCamera", 1, String.format("translatePos: dx=%f,dy=%f", dx, dy));

			return new Pose2d(xPos - dx/12, yPos + dy/12, Rotation2d.fromRadians(angleInRadians));
		}

		/*
		 * Computes position of the robot and updates the PoseEstimator
		 *
		 */
		private void updatePosition(
				DifferentialDrivePoseEstimator poseEstimator,
				long captureTime,
				Pose2d currentPose,
				int frameNo) {

			ApriltagLocation tag = ApriltagLocations.findTag(m_tag);

			// Logger.log("ApriltagsCamera", 1, String.format("no=%d, frame=%d ", m_tag, frameNo));

			if (tag != null) {
				double cx = m_tvec[0];
				double cz = m_tvec[2];

				m_angleInDegrees = m_relAngleInDegrees + tag.m_targetAngleDegrees - m_cameraAngleDegrees;

				SmartDashboard.putNumber("update relAngle", m_relAngleInDegrees);
				SmartDashboard.putNumber("update cam angle", m_cameraAngleDegrees);

				if (Math.abs(m_relAngleInDegrees) > 20)
				{
					return;
				}

				// Logger.log("ApriltagsCamera", 1, String.format("a=%f,r=%f,t=%f", m_angleInDegrees, m_relAngleInDegrees, tag.m_targetAngleDegrees));

				double a = Math.atan2(cx, cz);
				double b = Math.toRadians(m_angleInDegrees) - Math.PI / 2 - a;
				double d = Math.sqrt(cx * cx + cz * cz);

				SmartDashboard.putNumber("update d", d);

				if (d > 150)
				{
					return;
				}

				double dx = d * Math.sin(b);
				double dy = d * Math.cos(b);
				// Adjust for camera orientation
				double sin = Math.sin(Math.toRadians(m_cameraAngleDegrees));
				double cos = Math.cos(Math.toRadians(m_cameraAngleDegrees));
				SmartDashboard.putNumber("update angle", m_angleInDegrees);
				SmartDashboard.putNumber("update dx", dx);
				SmartDashboard.putNumber("update dy", dy);
				double dxp = dx * cos + dy * sin;
				double dyp = dy * cos - dx * sin;
				SmartDashboard.putNumber("update dxp", dxp);
				SmartDashboard.putNumber("update dyp", dyp);
				double xPos = (tag.m_xInches + dxp) / 12.0;
				double yPos = (tag.m_yInches - dyp) / 12.0;


				// Logger.log("ApriltagsCamera", 1, String.format("x=%f,y=%f,a=%f", xPos, yPos, m_angleInDegrees));

				Pose2d pos = translatePos2d(xPos, yPos, Math.toRadians(m_angleInDegrees));

				// Logger.log("ApriltagsCamera", 1, String.format("x=%f,y=%f,a=%f", pos.getX(), pos.getY(), m_angleInDegrees));

				double ex = pos.getX() - currentPose.getX();
				double ey = pos.getY() - currentPose.getY(); 
				boolean invalid;

				if ((Math.abs(ex) > 1) || (Math.abs(ey) > 1))
				{
					invalid = m_invalidCount < 3;
					Logger.log("ApriltagsCamera", 1, String.format("Spurious tag: id=%d fn=%d: %f,%f cnt=%d invalid=%b", m_tag, frameNo, ex, ey, m_invalidCount, invalid));
					m_invalidCount++;
				}
				else
				{
					invalid = false;
					m_invalidCount = 0;
				}

				if (!invalid)
				{
					pos = ParadoxField.pose2dFromParadox(pos);

					// Logger.log("ApriltagsCamera", 1, String.format("raw: b=%f,dx=%f,dy=%f,x=%f,y=%f,a=%f", Math.toDegrees(b), dx, dy, xPos, yPos, m_angleInDegrees));
					// Logger.log("ApriltagsCamera", 1, String.format("add: x=%f,y=%f,a=%f,t=%f", pos.getX(), pos.getY(), pos.getRotation().getDegrees(), convertTime(captureTime)));

					// There's an extended version of addVisionMeasurement() that takes stddevs directly as a third parameter.
					// E.g. VecBuilder.fill(0.1, 0.1, 0.1)
					// -Gavin
					poseEstimator.addVisionMeasurement(pos,	convertTime(captureTime));
					pos = ParadoxField.pose2dFromFRC(poseEstimator.getEstimatedPosition());
					// Logger.log("ApriltagsCamera", 1, String.format("est: x=%f,y=%f,a=%f,t=%f", pos.getX(), pos.getY(), pos.getRotation().getDegrees(), convertTime(captureTime)));
				}
			}
		}
	}

	int m_count;

	/**
	 * @brief The ApriltagsCameraRegions specifies list of detected regions
	 *
	 *        The list will contain up to the max regions specified using the
	 *        ImageViewer and will be sorted from the largest to smallest area.
	 *
	 */
	public class ApriltagsCameraRegions {
		public int m_targetVertPos; // !<Specifies the vertical target position as set by the ImageViewer
		public int m_targetHorzPos; // !<Specifies the horizontal target position as set by the ImageViewer
		public int m_frameNo; // !<Specifies the frame #
		public int m_width; // !<Specifies the width of the camera image (e.g. 640)
		public int m_height; // !<Specifies the height of the camera image (e.g. 480)
		public int m_lostFrames; // !<Specifies the number of lost frames
		public int m_profile; // Not used
		public long m_captureTime; // !<Specifies the time at which the image was acquired in ms
		public int m_procTime; // !<Specifies the time required to process this image in ms
		public int m_fps; // !<specifies the current frame rate in frames/sec
		public ArrayList<ApriltagsCameraRegion> m_regions = new ArrayList<ApriltagsCameraRegion>();

		// ! @cond PRIVATE
		protected ApriltagsCameraRegions(int frameNo, int targetVertPos, int targetHorzPos, int width, int height,
				int lostFrames,
				long captureTime, int procTime, int fps) {
			Logger.log("ApriltagsCameraRegions", -1,
					String.format("ApriltagsCameraRegions(): width = %d, height = %d", width, height));

			m_frameNo = frameNo;
			m_targetVertPos = targetVertPos;
			m_targetHorzPos = targetHorzPos;
			m_width = width;
			m_height = height;
			m_lostFrames = lostFrames;
			m_fps = fps;
			m_captureTime = captureTime;
			m_procTime = procTime;
		}

		protected void addRegion(int tag, double r0, double r1, double r2, double t0, double t1, double t2, double x0,
				double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
			m_regions.add(new ApriltagsCameraRegion(tag, r0, r1, r2, t0, t1, t2, x0, y0, x1, y1, x2, y2, x3, y3));
		}
		// ! @endcond

		/**
		 * Returns a count of the regions
		 * 
		 */
		public int getRegionCount() {
			return (m_regions.size());
		}

		/**
		 * Returns a specified region
		 * 
		 * @param region - Specifies the region to return
		 */
		public ApriltagsCameraRegion getRegion(int region) {

			if ((region >= 0) && (region < m_regions.size())) {

				return (m_regions.get(region));
			}
			;
			return (null);
		}

		/**
		 * Class for the return of the robot position
		 */
		public class RobotPos {
			public double m_x; // Absolute x position of the robot
			public double m_y; // Absolute y position of the robot

			public RobotPos(double x, double y) {
				m_x = x;
				m_y = y;
			}
		}

		/*
		 * Computes the absolute position of the robot using the infomation
		 * provided by the visible Apriltags
		 */
		// public RobotPos ComputeRobotPosition(ApriltagLocation[] tags, double yaw) {
		// double x = 0;
		// double y = 0;
		// int nRegions = m_regions.size();
		// int count = 0;

		// if (nRegions < 1) {
		// return (null);
		// }

		// for (int i = 0; i < nRegions; i++) {
		// ApriltagsCameraRegion region = getRegion(i);
		// ApriltagLocation tag = findTag(tags, region.m_tag);

		// if (tag != null) {
		// region.m_angleInDegrees = region.m_relAngleInDegrees +
		// tag.m_targetAngleDegrees;

		// RobotPos pos = computePosition(region, yaw);

		// x += tag.m_xInches + pos.m_x;
		// y += tag.m_yInches - pos.m_y;
		// count++;
		// }
		// }

		// return new RobotPos(x / count, y / count);
		// }

	}

	private static final int k_syncRetry = 5000;
	private static final int k_syncFirst = 1000;

	private Network m_network = null;
	private ApriltagsCameraRegions m_regions = null;
	private ApriltagsCameraRegions m_nextRegions = null;
	private long m_syncTime;
	private int m_averageDelayCount = 0;
	private int m_averageDelayMax = 30;
	private int m_averageDelaySum = 0;
	private int m_averageDelay = 0;
	private int m_maxDelay = 0;
	private int m_minDelay = Integer.MAX_VALUE;
	private int m_lostFrames;
	private int m_lastLostFrame = 0;
	private long m_startTime = 0;
	private boolean m_connected = false;
	private Timer m_watchdogTimer = new Timer();
	private long m_lastMessage;
	private static final int k_timeout = 5000;
	// private double m_angleOffsetInDegrees = 0;
	boolean m_angleOffsetInitialized = false;
	private final double m_xOffsetInches;
	private final double m_yOffsetInches;
	private final double m_cameraAngleDegrees;
	private int m_invalidCount;

	private static long k_timeOffset = 0;

	public static double convertTime(long time) {
		if (k_timeOffset == 0) {
			k_timeOffset = System.currentTimeMillis();
		}

		return (time - k_timeOffset) / 1000.0;
	}

	public static double getTime() {
		return (convertTime(System.currentTimeMillis()));
	}

	public ApriltagsCamera(double xOffsetInches, double yOffsetInches, double cameraAngleDegrees) {
		m_xOffsetInches = xOffsetInches;
		m_yOffsetInches = yOffsetInches;
		m_cameraAngleDegrees = cameraAngleDegrees;
		m_watchdogTimer.scheduleAtFixedRate(new TimerTask() {

			@Override
			public void run() {
				if (m_connected) {
					Logger.log("ApriltagsCamera", -1, "WatchDog");

					m_network.sendMessage("k");

					if (m_lastMessage + k_timeout < System.currentTimeMillis()) {
						Logger.log("ApriltagsCamera", 3, "Network timeout");
						m_network.closeConnection();
					}
				}
			}
		}, 200, 200);
	}

	/**
	 * Returns true if connected to the camera
	 * 
	 */
	public boolean isConnected() {
		return (m_connected);
	}

	// ! @cond PRIVATE
	public static int[] parseIntegers(String str, int count) {
		int[] args = new int[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Integer.parseInt(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}

	public static long[] parseLong(String str, int count) {
		long[] args = new long[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Long.parseLong(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}

	public static double[] parseDouble(String str, int count) {
		double[] args = new double[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Double.parseDouble(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}

	long m_pingTime = 0;

	public void Ping() {
		m_pingTime = getTimeMs();

		m_network.sendMessage("p");
	}
	// ! @endcond

	/**
	 * Connects to the Raspberry Pi Data server
	 *
	 * @param host - Specifies IP for the host
	 * @param port - Specifies the port (default is 5800)
	 */
	public void connect(String host, int port) {
		m_network = new Network();

		m_startTime = System.currentTimeMillis();

		m_network.connect(this, host, port);

	}

	private void timeSync() {
		long time = getTimeMs();
		if (time > m_syncTime) {
			Logger.log("ApriltagCameras", -1, "TimeSync()");

			m_network.sendMessage(String.format("T1 %d", getTimeMs()));

			m_syncTime = time + k_syncRetry;
		}

	}

	private void processCameraFrame(String args) {
		long a[] = parseLong(args, 9);

		// System.out.println(String.format("%d %s", System.currentTimeMillis(), args));

		if (a != null) {
			m_nextRegions = new ApriltagsCameraRegions((int) a[0], (int) a[1], (int) a[2], (int) a[3], (int) a[4],
					(int) a[5],
					a[6], (int) a[7], (int) a[8]);

			int delay = (int) (System.currentTimeMillis() - m_nextRegions.m_captureTime);
			int averageDelay = -1;

			m_averageDelaySum += delay;
			if (++m_averageDelayCount >= m_averageDelayMax) {
				averageDelay = m_averageDelaySum / m_averageDelayCount;

				m_averageDelayCount = 0;
				m_averageDelaySum = 0;
			}

			synchronized (this) {
				if (averageDelay > 0) {
					m_averageDelay = averageDelay;
				}

				if (delay > m_maxDelay) {
					m_maxDelay = delay;
				}
				if (delay < m_minDelay) {
					m_minDelay = delay;
				}

				m_lostFrames = (int) a[5];
			}
		}
	}

	/**
	 * Returns the current camera performance stats
	 *
	 */
	public ApriltagsCameraStats getStats() {
		synchronized (this) {
			return (new ApriltagsCameraStats(m_averageDelay, m_maxDelay, m_minDelay, m_lostFrames - m_lastLostFrame,
					System.currentTimeMillis() - m_startTime));
		}
	}

	/**
	 * Clears the camera performance stats
	 *
	 */
	public void clearStats() {
		synchronized (this) {
			m_maxDelay = 0;
			m_minDelay = Integer.MAX_VALUE;
			m_lostFrames = 0;
			m_startTime = System.currentTimeMillis();
		}
	}

	private void processCameraRegion(String args) {
		double a[] = parseDouble(args, 15);

		if ((a != null) && (m_nextRegions != null)) {
			m_nextRegions.addRegion((int) a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11],
					a[12], a[13], a[14]);
		}
	}

	private void processCameraEnd(String args) {
		synchronized (this) {
			m_regions = m_nextRegions;
			m_nextRegions = null;
		}

		timeSync();
	}

	/**
	 * Returns the latest set of camera regions. Note that the data is received from
	 * the camera in a separate thread so calling this twice in a row can generate a
	 * different result. Once you retrieve and instance of ApriltagsCameraRegions
	 * however, you can be assured that it will NOT be modified when a new frame is
	 * received.
	 *
	 */
	public ApriltagsCameraRegions getRegions() {
		synchronized (this) {
			return (m_regions);
		}

	}

	private long getTimeMs() {
		return System.currentTimeMillis();
	}

	private void processTimeSync() {
		Logger.log("ApriltagCamera", -1, "ProcessTimeSync()");

		long time = getTimeMs();

		m_network.sendMessage(String.format("T2 %d", time));
	}

	// ! @cond PRIVATE
	@Override
	public void processData(String data) {
		Logger.log("ApriltagCamera", -1, String.format("Data: %s", data));

		m_lastMessage = System.currentTimeMillis();

		switch (data.charAt(0)) {
			case 'F':
				processCameraFrame(data.substring(1).trim());
				break;

			case 'R':
				processCameraRegion(data.substring(1).trim());
				break;

			case 'E':
				processCameraEnd(data.substring(1).trim());
				break;

			case 'p':
				Logger.log("ApriltagsCamera", 3, String.format("Ping = %d", getTimeMs() - m_pingTime));
				break;

			case 'T': // sync
				processTimeSync();
				break;

			default:
				Logger.log("ApriltagsCamera", 3, String.format("Invalid command: %s", data));
				break;
		}
	}

	@Override
	public void disconnected() {
		m_connected = false;
	}

	@Override
	public void connected() {
		m_connected = true;

		m_syncTime = getTimeMs() + k_syncFirst;
		m_lastMessage = m_syncTime;
	}
	// ! @endcond

	int m_frameCount = 0;
	int m_missingCount = 0;
	int m_lastFrame = -1;
	boolean m_logTags = true;

	public void processRegions(DifferentialDrivePoseEstimator poseEstimator) {
		ApriltagsCameraRegions regions = getRegions();

		if ((regions != null) && (regions.m_frameNo != m_lastFrame)) {
			// Logger.log("ApriltagsCamera", 1, String.format("processRegions: no=%d,last=%d,cnt=%d", regions.m_frameNo, m_lastFrame, regions.m_regions.size()));
			m_lastFrame = regions.m_frameNo;

			Pose2d currentPose = ParadoxField.pose2dFromFRC(poseEstimator.getEstimatedPosition());

			for (ApriltagsCameraRegion region : regions.m_regions)
			{
				// Logger.log("ApriltagsCamera", 1, "Calling updatePosition");
				region.updatePosition(poseEstimator, regions.m_captureTime, currentPose, regions.m_frameNo);
			}

			if (m_logTags) {
				Logger.log("Robot", -1, String.format("nRegions = %d", regions.m_regions.size()));
				SmartDashboard.putNumber("cam: nRegions", regions.m_regions.size());
				SmartDashboard.putNumber("cam: nFrames", ++m_frameCount);
				SmartDashboard.putNumber("cam: Delay", System.currentTimeMillis() - regions.m_captureTime);
				SmartDashboard.putNumber("cam: FPS", regions.m_fps);

				if (regions.m_regions.size() == 0) {
					SmartDashboard.putNumber("missing", ++m_missingCount);
				}

				for (ApriltagsCameraRegion region : regions.m_regions) {
					if (region.m_tag == 2) {

						SmartDashboard.putNumber(String.format("cam: Tag%d", region.m_tag), region.m_tag);
						SmartDashboard.putNumber(String.format("cam: Dist%d", region.m_tag),
								Math.sqrt(region.m_tvec[0] * region.m_tvec[0] +
										region.m_tvec[1] * region.m_tvec[1] +
										region.m_tvec[2] * region.m_tvec[2]));
						SmartDashboard.putNumber(String.format("cam: rx%d", region.m_tag), region.m_rvec[0]);
						SmartDashboard.putNumber(String.format("cam: ry%d", region.m_tag), region.m_rvec[1]);
						SmartDashboard.putNumber(String.format("cam: rz%d", region.m_tag), region.m_rvec[2]);
						SmartDashboard.putNumber(String.format("cam: tx%d", region.m_tag), region.m_tvec[0]);
						SmartDashboard.putNumber(String.format("cam: ty%d", region.m_tag), region.m_tvec[1]);
						SmartDashboard.putNumber(String.format("cam: tz%d", region.m_tag), region.m_tvec[2]);
						SmartDashboard.putNumber(String.format("cam: angle%d", region.m_tag),
								ParadoxField.normalizeAngle(region.m_angleInDegrees));
						SmartDashboard.putNumber(String.format("cam: relAngle%d", region.m_tag),
								ParadoxField.normalizeAngle(region.m_relAngleInDegrees));
						SmartDashboard.putNumber(String.format("cam: offset%d", region.m_tag), region.m_angleOffset);
					}
				}
			}
		}
	}
}
