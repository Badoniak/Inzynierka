using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine;



public class RosVelocityController : MonoBehaviour
{
    ROSConnection ros;

    [Header("ROS Topic")]
    public string cmdVelTopic = "/diff_drive_controller/cmd_vel";

    [Header("Wheel Articulation Bodies")]
    public ArticulationBody frontLeftWheel;
    public ArticulationBody frontRightWheel;
    public ArticulationBody backLeftWheel;
    public ArticulationBody backRightWheel;

    [Header("Robot Parameters")]
    public float wheelRadius = 0.26f;
    public float wheelSeparation = 1.17f; // distance between left and right wheels

    private float linearVelocity = 0f;
    private float angularVelocity = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(cmdVelTopic, CmdVelCallback);
    }

    void CmdVelCallback(TwistMsg msg)
    {
        linearVelocity = (float)msg.linear.x;
        angularVelocity = (float)msg.angular.z;
    }

    void FixedUpdate()
    {
        float leftWheelVel = (linearVelocity - angularVelocity * wheelSeparation / 2f) / wheelRadius;
        float rightWheelVel = (linearVelocity + angularVelocity * wheelSeparation / 2f) / wheelRadius;

        SetWheelVelocity(frontLeftWheel, leftWheelVel);
        SetWheelVelocity(backLeftWheel, leftWheelVel);
        SetWheelVelocity(frontRightWheel, rightWheelVel);
        SetWheelVelocity(backRightWheel, rightWheelVel);
    }

    void SetWheelVelocity(ArticulationBody wheel, float velocity)
    {
        if (wheel == null) return;
        var drive = wheel.xDrive;
        drive.targetVelocity = velocity * Mathf.Rad2Deg; // Unity expects deg/s
        wheel.xDrive = drive;
    }
}
