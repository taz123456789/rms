using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
public class roswithcontrol : MonoBehaviour
{
    private ROSConnection ros;
    public string state="/arm/joint_states";
    public string command="/arm/joint_command";
    public string target="/arm/ik_target";//inverse kinematics
    public jointarmcontrol armcontrol;
    private jointstatemsg statemsg;
    private string[] jointNames={"j1z","j1x","j2z","j2x","j3z","j3x"};
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        ros=ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<jointstatemsg>(state);
        ros.Subscribe<jointstatemsg>(command);//////callback is needed
        ros.Subscribe<PoseMsg>(target);///callback 
        statemsg=new jointstatemsg{
            name=jointNames,
            position=new double[6],
            velocity=new double[6],
            effort=new double[6]
        };
        InvokeRepeating("Publisher",1.0f,0.1f);//periodically publishing(10hz)

    }

    // Update is called once per frame
    void Update()
    {
        UpdateStateMsg();
    }
    void UpdateStateMsg(){

    }
    void Publisher(){
        ros.Publish(state,statemsg);
    } 
}
