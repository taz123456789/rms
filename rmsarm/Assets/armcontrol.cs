using UnityEngine;

public class ThreeJointArmController : MonoBehaviour
{
    [System.Serializable]
    public class JointControl
    {
        public Transform joint;               // The joint's transform (empty GameObject)
        [Range(0, 360)] public float angleZ;  // Rotation around local Z axis (0–360°)
        [Range(0, 120)] public float angleX;  // Rotation around local X axis (0–120°)
    }

    public JointControl joint1;
    public JointControl joint2;
    public JointControl joint3;

    void Update()
    {
        // Apply rotations to each joint.
        // Quaternion.Euler(angleX, 0, angleZ) applies Z rotation first, then X rotation.
        if (joint1.joint != null)
            joint1.joint.localRotation = Quaternion.Euler(joint1.angleX, 0, joint1.angleZ);

        if (joint2.joint != null)
            joint2.joint.localRotation = Quaternion.Euler(joint2.angleX, 0, joint2.angleZ);

        if (joint3.joint != null)
            joint3.joint.localRotation = Quaternion.Euler(joint3.angleX, 0, joint3.angleZ);
    }
}