using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Reflection;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// NPC Vehicle class.
    /// Controlled by Position and Rotation.
    /// </summary>
    ///
    public class NPC : MonoBehaviour
    {
        public unique_identifier_msgs.msg.UUID rosuuid;
        public virtual Vector3 Velocity { get; }
        public virtual Vector3 AngularVelocity { get; }

        public void SetUUID(){
        Guid guid = Guid.NewGuid();
        rosuuid = new unique_identifier_msgs.msg.UUID();
        PropertyInfo prop = rosuuid.GetType().GetProperty("Uuid", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static);
        prop.SetValue(rosuuid, guid.ToByteArray());
        
    }
    }

}