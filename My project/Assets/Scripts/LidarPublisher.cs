using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

[RequireComponent(typeof(LidarSensor2D))]
public class LidarPublisher : MonoBehaviour
{
    public string topicName = "/LaserScan";
    private ROSConnection ros;
    private LidarSensor2D lidar;

    [Header("LaserScan Settings")]
    public float angleMin = 0f;
    public float angleMax = 2 * Mathf.PI; // pełne 360°
    public float rangeMin = 0.05f;
    public float rangeMax = 10f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);

        lidar = GetComponent<LidarSensor2D>();
    }

    void Update()
    {
        PublishLaserScan();
    }

    void PublishLaserScan()
    {
        LaserScanMsg msg = new LaserScanMsg();

        msg.header.frame_id = "lidar_link";
        msg.angle_min = angleMin;
        msg.angle_max = angleMax;
        msg.angle_increment = (angleMax - angleMin) / lidar.numRays;
        msg.time_increment = 0f;
        msg.scan_time = 1f / lidar.scanFrequency;
        msg.range_min = rangeMin;
        msg.range_max = rangeMax;
        msg.ranges = lidar.ranges;
        msg.intensities = lidar.intensities;

        ros.Publish(topicName, msg);
    }
}
