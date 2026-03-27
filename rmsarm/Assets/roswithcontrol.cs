using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using System.Collections;

public class roswithcontrol : MonoBehaviour
{
    private ROSConnection ros;
    public string state = "/arm/joint_states";
    public string command = "/arm/joint_command";
    public string eepose = "/arm/ee_pose"; //end effector
    public string target = "/arm/ik_target"; //inverse kinematics
    public jointarmcontrol armcontrol; //arm controller script
    public Transform armBase;
    public Transform endeffector;
    private JointStateMsg statemsg;
    public float publishRate = 20f;
    //limits
    private const float Z_MIN = 0f;
    private const float Z_MAX = 360f;
    private const float X_MIN = -80f;
    private const float X_MAX = 80f;
    
    private string[] jointNames = { "j1z", "j1x", "j2z", "j2x", "j3z", "j3x" };
    
    // Start is called once before the first execution of Update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        //publishers
        ros.RegisterPublisher<JointStateMsg>(state);
        ros.RegisterPublisher<PoseMsg>(eepose);
        ros.RegisterPublisher<JointStateMsg>(command);
        //subscribers
        ros.Subscribe<JointStateMsg>(command, commandCallback);
        ros.Subscribe<PoseMsg>(target, targetCallback);
        
        //init
        statemsg = new JointStateMsg
        {
            header = new HeaderMsg(),
            name = jointNames,
            position = new double[6],
            velocity = new double[6],
            effort = new double[6]
        };
        
        //periodic publishing
        InvokeRepeating("Publisher", 1.0f, 1.0f / publishRate);
        
        Debug.Log("ROS2 Arm Controller initialized");
    }
    
    void Update()
    {
        UpdateStateMsg();
        // if (Input.GetKeyDown(KeyCode.W))
        // {
        //     SendIKTarget(new Vector3(1.5f, 4.2f, 0.3f), Quaternion.identity);
        // }
        
        // if (Input.GetKeyDown(KeyCode.S))
        // {
        //     ResetToZero();
        // }
         //ros.Publish(command, statemsg);
    }
    
    void UpdateStateMsg()
    {
        if (armcontrol == null) return;
        //degrees -> radians 
        statemsg.position[0] = armcontrol.joint1.angleZ * Mathf.Deg2Rad;  // j1z
        statemsg.position[1] = armcontrol.joint1.angleX * Mathf.Deg2Rad;  // j1x
        statemsg.position[2] = armcontrol.joint2.angleZ * Mathf.Deg2Rad;  // j2z
        statemsg.position[3] = armcontrol.joint2.angleX * Mathf.Deg2Rad;  // j2x
        statemsg.position[4] = armcontrol.joint3.angleZ * Mathf.Deg2Rad;  // j3z
        statemsg.position[5] = armcontrol.joint3.angleX * Mathf.Deg2Rad;  // j3x
        //header timestamp
        double seconds = Time.time;
        int sec = (int)seconds;
        uint nanosec = (uint)((seconds - sec) * 1e9);
        statemsg.header.stamp.sec = sec;
        statemsg.header.stamp.nanosec = nanosec;
        statemsg.header.frame_id = "";
    }
    
    void Publisher()
    {
        if (armcontrol == null) return;
        ros.Publish(state, statemsg);
        //end effector pose
        PublishEEPose();
    }
    
    void PublishEEPose()
{
    if (armBase == null || endeffector == null) return;
    
    // Get position relative to base
    Vector3 localPos = armBase.InverseTransformPoint(endeffector.position);
    Quaternion localRot = Quaternion.Inverse(armBase.rotation) * endeffector.rotation;
    
    // Convert Unity coordinate system to ROS
    // Unity: X right, Y up, Z forward
    // ROS:   X forward, Y left, Z up
    // Mapping: Unity (x, y, z) -> ROS (z, -x, y)
    Vector3 rosPos = new Vector3(localPos.z, -localPos.x, localPos.y);
    
    // Convert quaternion (simplified - you may need to adjust based on your robot's orientation)
    Quaternion rosRot = new Quaternion(-localRot.z, localRot.x, -localRot.y, localRot.w);
    
    PoseMsg poseMsg = new PoseMsg
    {
        position = new PointMsg(rosPos.x, rosPos.y, rosPos.z),
        orientation = new QuaternionMsg(rosRot.x, rosRot.y, rosRot.z, rosRot.w)
    };
    
    ros.Publish(eepose, poseMsg);
    
    // Debug output
    Debug.Log($"Published EE Pose - Unity local: ({localPos.x:F3}, {localPos.y:F3}, {localPos.z:F3}) -> ROS: ({rosPos.x:F3}, {rosPos.y:F3}, {rosPos.z:F3})");
}
    
    void commandCallback(JointStateMsg msg)
    {
        Debug.Log("✅ JOINT COMMAND RECEIVED IN UNITY!");
        if (armcontrol == null)
        {
            Debug.LogWarning("controller not assigned");
            return;
        }
        
        if (msg.position.Length < 6)
        {
            Debug.LogWarning("joint command has insufficient positions");
            return;
        }
        
        Debug.Log("received joint command");
         // Order: [j1z, j1x, j2z, j2x, j3z, j3x]
        armcontrol.joint1.angleZ = ClampAngle((float)msg.position[0] * Mathf.Rad2Deg, Z_MIN, Z_MAX);
        armcontrol.joint1.angleX = ClampAngle((float)msg.position[1] * Mathf.Rad2Deg, X_MIN, X_MAX);
        armcontrol.joint2.angleZ = ClampAngle((float)msg.position[2] * Mathf.Rad2Deg, Z_MIN, Z_MAX);
        armcontrol.joint2.angleX = ClampAngle((float)msg.position[3] * Mathf.Rad2Deg, X_MIN, X_MAX);
        armcontrol.joint3.angleZ = ClampAngle((float)msg.position[4] * Mathf.Rad2Deg, Z_MIN, Z_MAX);
        armcontrol.joint3.angleX = ClampAngle((float)msg.position[5] * Mathf.Rad2Deg, X_MIN, X_MAX);
    }
  void targetCallback(PoseMsg msg)
{
    if (armBase == null) return;
    
    // Convert ROS position to Unity local position
    // ROS (x, y, z) -> Unity local (-y, z, x)
    Vector3 rosPos = new Vector3((float)msg.position.x, (float)msg.position.y, (float)msg.position.z);
    Vector3 unityLocalPos = new Vector3(-rosPos.y, rosPos.z, rosPos.x);
    
    // Convert to world position
    Vector3 worldPos = armBase.TransformPoint(unityLocalPos);
    
    // Convert ROS quaternion to Unity
    Quaternion rosRot = new Quaternion(
        (float)msg.orientation.x,
        (float)msg.orientation.y,
        (float)msg.orientation.z,
        (float)msg.orientation.w
    );
    
    // Convert: ROS quaternion to Unity (inverse of the mapping used in PublishEEPose)
    Quaternion unityLocalRot = new Quaternion(-rosRot.y, rosRot.z, -rosRot.x, rosRot.w);
    Quaternion worldRot = armBase.rotation * unityLocalRot;
    
    Debug.Log($"Received IK target - ROS: ({rosPos.x:F3}, {rosPos.y:F3}, {rosPos.z:F3}) -> Unity world: ({worldPos.x:F3}, {worldPos.y:F3}, {worldPos.z:F3})");
    
    // Draw target marker
    Debug.DrawLine(armBase.position, worldPos, Color.green, 2.0f);
    DrawTargetMarker(worldPos);
}

    
    void DrawTargetMarker(Vector3 position)
    {//for now
        Debug.DrawLine(position + Vector3.up * 0.1f, position - Vector3.up * 0.1f, Color.green, 1.0f);
        Debug.DrawLine(position + Vector3.right * 0.1f, position - Vector3.right * 0.1f, Color.green, 1.0f);
        Debug.DrawLine(position + Vector3.forward * 0.1f, position - Vector3.forward * 0.1f, Color.green, 1.0f);
    }
    
    public void SendIKTarget(Vector3 position, Quaternion orientation)
{
    if (armBase == null) return;
    
    // Convert Unity world position to local position relative to base
    Vector3 localPos = armBase.InverseTransformPoint(position);
    Quaternion localRot = Quaternion.Inverse(armBase.rotation) * orientation;
    
    // Convert Unity local coordinates to ROS coordinates
    // Unity (x, y, z) -> ROS (z, -x, y)
    Vector3 rosPos = new Vector3(localPos.z, -localPos.x, localPos.y);
    Quaternion rosRot = new Quaternion(-localRot.z, localRot.x, -localRot.y, localRot.w);
    
    PoseMsg poseMsg = new PoseMsg
    {
        position = new PointMsg(rosPos.x, rosPos.y, rosPos.z),
        orientation = new QuaternionMsg(rosRot.x, rosRot.y, rosRot.z, rosRot.w)
    };
    
    ros.Publish(target, poseMsg);
    Debug.Log($"IK target sent - Unity world: ({position.x:F2}, {position.y:F2}, {position.z:F2}) -> ROS: ({rosPos.x:F2}, {rosPos.y:F2}, {rosPos.z:F2})");
}
    private float ClampAngle(float angle, float min, float max)
    {
        if (min == 0 && max == 360)
        {
            angle = angle % 360f;
            if (angle < 0) angle += 360f;
            return angle;
        }
        else
        {
            return Mathf.Clamp(angle, min, max);
        }
    }

    public void SetJointAngles(float[] angles)
    {
        if (angles.Length < 6 || armcontrol == null) return;
        
        armcontrol.joint1.angleZ = ClampAngle(angles[0], Z_MIN, Z_MAX);
        armcontrol.joint1.angleX = ClampAngle(angles[1], X_MIN, X_MAX);
        armcontrol.joint2.angleZ = ClampAngle(angles[2], Z_MIN, Z_MAX);
        armcontrol.joint2.angleX = ClampAngle(angles[3], X_MIN, X_MAX);
        armcontrol.joint3.angleZ = ClampAngle(angles[4], Z_MIN, Z_MAX);
        armcontrol.joint3.angleX = ClampAngle(angles[5], X_MIN, X_MAX);
    }
  
    public float[] GetJointAngles()
    //current joint angles in degrees
    {
        if (armcontrol == null) return new float[6];
        
        return new float[]
        {
            armcontrol.joint1.angleZ,
            armcontrol.joint1.angleX,
            armcontrol.joint2.angleZ,
            armcontrol.joint2.angleX,
            armcontrol.joint3.angleZ,
            armcontrol.joint3.angleX
        };
    }
    
    public double[] GetJointAnglesRad()
    {//in rad
        float[] deg = GetJointAngles();
        double[] rad = new double[6];
        for (int i = 0; i < 6; i++)
        {
            rad[i] = deg[i] * Mathf.Deg2Rad;
        }
        return rad;
    }
    
    public void ResetToZero()
    {
        if (armcontrol == null) return;
        armcontrol.joint1.angleZ = 0;
        armcontrol.joint1.angleX = 0;
        armcontrol.joint2.angleZ = 0;
        armcontrol.joint2.angleX = 0;
        armcontrol.joint3.angleZ = 0;
        armcontrol.joint3.angleX = 0;
        Debug.Log("Arm reset to zero position");
    }
    
    void OnDrawGizmos()
    {
        if (armBase != null && endeffector != null)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(armBase.position, endeffector.position);
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(endeffector.position, 0.05f);
        }
 
    }
}