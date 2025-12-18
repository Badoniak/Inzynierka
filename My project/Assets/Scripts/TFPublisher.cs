using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using System;
using RosMessageTypes.BuiltinInterfaces;

public class TFPublisher : MonoBehaviour
{
    public string parentFrame = "map";
    public string childFrame = "odom";
    public float publishRate = 10f;

    private ROSConnection ros;
    private float timeSinceLastPublish = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>("/tf");
    }

    void Update()
    {
        timeSinceLastPublish += Time.deltaTime;
        if (timeSinceLastPublish < 1f / publishRate) return;
        timeSinceLastPublish = 0f;

        PublishTF();
    }

    void PublishTF()
    {
        // Konwersja pozycji z Unity (Y=up, Z=forward) → ROS (Z=up, X=forward)
        Vector3 unityPos = transform.position;
        Vector3 rosPos = new Vector3(unityPos.z, -unityPos.x, unityPos.y);

        // Konwersja rotacji z Unity → ROS
        Quaternion unityRot = transform.rotation;
        Quaternion rosRot = new Quaternion(
            unityRot.z,
            -unityRot.x,
            unityRot.y,
            -unityRot.w
        );

        TransformStampedMsg tf = new TransformStampedMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg((int)Time.time, (uint)((Time.time - (uint)Time.time) * 1e9)),
                frame_id = parentFrame
            },
            child_frame_id = childFrame,
            transform = new TransformMsg
            {
                translation = new Vector3Msg(rosPos.x, rosPos.y, rosPos.z),
                rotation = new QuaternionMsg(rosRot.x, rosRot.y, rosRot.z, rosRot.w)
            }
        };

        TFMessageMsg tfMessage = new TFMessageMsg(new TransformStampedMsg[] { tf });
        ros.Publish("/tf", tfMessage);
    }
}
