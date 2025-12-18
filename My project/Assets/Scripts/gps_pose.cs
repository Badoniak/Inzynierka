using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class OdomPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/odom";
    public string frameId = "odom";
    public string childFrameId = "base_link";
    public float publishRate = 10f;

    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1.0f / publishRate)
        {
            PublishOdom();
            timeElapsed = 0;
        }
    }

    void PublishOdom()
    {
        OdometryMsg odom = new OdometryMsg();

        // Nagłówek
        odom.header.frame_id = frameId;
        odom.header.stamp = new TimeMsg((int)Time.time, (uint)((Time.time - (int)Time.time) * 1e9));
        odom.child_frame_id = childFrameId;

        // Konwersja pozycji Unity -> ROS (FLU - Forward, Left, Up)
        // Unity (Z=forward, X=right, Y=up) -> ROS (X=forward, Y=left, Z=up)
        var unityPos = transform.position;
        odom.pose.pose.position.x = unityPos.z; 
        odom.pose.pose.position.y = -unityPos.x;
        odom.pose.pose.position.z = 0; // Zakładamy płaską podłogę

        // Konwersja rotacji
        var unityRot = transform.rotation;
        odom.pose.pose.orientation.x = unityRot.z;
        odom.pose.pose.orientation.y = -unityRot.x;
        odom.pose.pose.orientation.z = unityRot.y;
        odom.pose.pose.orientation.w = -unityRot.w;

        // (Opcjonalnie) Prędkości można zostawić 0, jeśli regulator patrzy tylko na pozycję

        ros.Publish(topicName, odom);
    }
}