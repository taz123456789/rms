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
    private const float X_MIN = -60f;
    private const float X_MAX = 60f;
    
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
         ros.Publish(command, statemsg);
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
        //Unity pose -> ROS pose 
        Vector3 localPos = armBase.InverseTransformPoint(endeffector.position);
        Quaternion localRot = Quaternion.Inverse(armBase.rotation) * endeffector.rotation;
        PoseMsg poseMsg = new PoseMsg
        {
            position = new PointMsg(localPos.x, localPos.y, localPos.z),
            orientation = new QuaternionMsg(localRot.x, localRot.y, localRot.z, localRot.w)
        };
        ros.Publish(eepose, poseMsg);
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
        if (armBase == null)
        {
            Debug.LogWarning(" base not assigned!");
            return;
        }
        
        //ROS pose -> Unity
        Vector3 targetPos = new Vector3(
            (float)msg.position.x,
            (float)msg.position.y,
            (float)msg.position.z
        );
        
        Quaternion targetRot = new Quaternion(
            (float)msg.orientation.x,
            (float)msg.orientation.y,
            (float)msg.orientation.z,
            (float)msg.orientation.w
        );
        // Transform  (ROS base frame -> Unity)
        Vector3 worldPos = armBase.TransformPoint(targetPos);
        Quaternion worldRot = armBase.rotation * targetRot;
        Debug.Log($"Received IK target: {worldPos}");
        //drawing target
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
        Vector3 localPos = armBase.InverseTransformPoint(position);
        Quaternion localRot = Quaternion.Inverse(armBase.rotation) * orientation;
        PoseMsg poseMsg = new PoseMsg
        {
            position = new PointMsg(localPos.x, localPos.y, localPos.z),
            orientation = new QuaternionMsg(localRot.x, localRot.y, localRot.z, localRot.w)
        };
        
        ros.Publish(target, poseMsg);
        Debug.Log($"IK target sent: ({position.x:F2}, {position.y:F2}, {position.z:F2})");
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