using UnityEngine;

[System.Serializable]
public class JointData
{
    public Transform joint;
    [Range(0, 360)] public float angleZ;
    [Range(-60, 60)] public float angleX;
}

public class jointarmcontrol : MonoBehaviour
{
    public JointData joint1;
    public JointData joint2;
    public JointData joint3;

    void Update()
    {
        // rotation(Z then X )
        if (joint1.joint != null)
            joint1.joint.localRotation = Quaternion.Euler(joint1.angleX, 0, joint1.angleZ);
        
        if (joint2.joint != null)
            joint2.joint.localRotation = Quaternion.Euler(joint2.angleX, 0, joint2.angleZ);
        
        if (joint3.joint != null)
            joint3.joint.localRotation = Quaternion.Euler(joint3.angleX, 0, joint3.angleZ);
    }
    public void ResetJoints()
    {
        joint1.angleZ = 0;
        joint1.angleX = 0;
        joint2.angleZ = 0;
        joint2.angleX = 0;
        joint3.angleZ = 0;
        joint3.angleX = 0;
    }

    public float[] GetJointAngles()
    {//as array
        return new float[]
        {
            joint1.angleZ,
            joint1.angleX,
            joint2.angleZ,
            joint2.angleX,
            joint3.angleZ,
            joint3.angleX
        };
    }
}