using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using AWSIM;

public class GnssNavSat : MonoBehaviour
{

  public float DEFAULT_REFERENCE_LATITUDE = 59.664230f;
  public float DEFAULT_REFERENCE_LONGITUDE = 10.762824f;
  public float DEFAULT_REFERENCE_ALTITUDE = 0.0f;
  public float DEFAULT_REFERENCE_HEADING = 0.0f;

  public float gaussian_deviation_x = 0.4f;
  public float gaussian_deviation_y = 0.4f;
  public float gaussian_deviation_z = 0.0f;

  const float equatorialRadius = 6378137.0f;
  const float flattening = 1.0f / 298.257223563f;
  const float excentricity2 = 2.0f * flattening - flattening * flattening;

  bool STATUS_FIX = true;
  bool STATUS_SBAS_FIX = false;
  bool STATUS_GBAS_FIX = false;
  bool SERVICE_GPS = true;
  bool SERVICE_GLONASS = true;
  bool SERVICE_COMPASS = true;
  bool SERVICE_GALILEO = true;

  public string frame_id_ = "gps_link";
  public string gps_topic_ = "/fix";
  public string vel_topic_ = "/vel";

  float referenceLatitude_;
  float referenceLongitude_;
  float referenceAltitude_;
  float referenceHeading_;
  float radius_north_;
  float radius_east_;

  sensor_msgs.msg.NavSatFix gps_msg_;
  geometry_msgs.msg.Vector3Stamped vel_msg_;

  IPublisher<sensor_msgs.msg.NavSatFix> fixPublisher;
  IPublisher<geometry_msgs.msg.Vector3Stamped> velPublisher;
  public QoSSettings qosSettings;

  System.Random random = new System.Random();

  Vector3 previousPosition = new Vector3(0, 0, 0);

  // Start is called before the first frame update
  void Start()
  {
    // add namespace to topic name
    string objname = gameObject.transform.root.name;

    gps_topic_ = objname + gps_topic_;
    vel_topic_ = objname + vel_topic_;

    gps_msg_ = new sensor_msgs.msg.NavSatFix();
    vel_msg_ = new geometry_msgs.msg.Vector3Stamped();

    // Set reference position.
    referenceLatitude_ = DEFAULT_REFERENCE_LATITUDE;
    referenceLongitude_ = DEFAULT_REFERENCE_LONGITUDE;
    referenceAltitude_ = DEFAULT_REFERENCE_ALTITUDE;
    referenceHeading_ = DEFAULT_REFERENCE_HEADING;

    // Create msg.
    gps_msg_ = new sensor_msgs.msg.NavSatFix();
    gps_msg_.Header.Frame_id = frame_id_;
    gps_msg_.Status.Service = sensor_msgs.msg.NavSatStatus.SERVICE_GPS;
    gps_msg_.Status.Status = sensor_msgs.msg.NavSatStatus.STATUS_FIX;

    // calculate earth radii
    float temp = 1.0f /
      (1.0f - excentricity2 * Mathf.Sin(referenceLatitude_ * Mathf.PI / 180.0f) *
      Mathf.Sin(referenceLatitude_ * Mathf.PI / 180.0f));
    float prime_vertical_radius = equatorialRadius * Mathf.Sqrt(temp);
    radius_north_ = prime_vertical_radius * (1.0f - excentricity2) * temp;
    radius_east_ = prime_vertical_radius * Mathf.Cos(referenceLatitude_ * Mathf.PI / 180.0f);

    var qos = qosSettings.GetQoSProfile();
    fixPublisher = SimulatorROS2Node.CreatePublisher<sensor_msgs.msg.NavSatFix>(gps_topic_, qos);
    velPublisher = SimulatorROS2Node.CreatePublisher<geometry_msgs.msg.Vector3Stamped>(vel_topic_, qos);

    previousPosition = ROS2.Transformations.Unity2Ros(GameObject.Find("base_link").transform.position);
  }

  // Update is called once per frame
  void Update()
  {
    // Publish GPS data.
    Vector3 position;
    Quaternion rotation;
    GameObject.Find("base_link").transform.GetPositionAndRotation(out position, out rotation);
    position = ROS2.Transformations.Unity2Ros(position);

    Vector3 velocity = (position - previousPosition) / Time.deltaTime;
    previousPosition = position;

    // Update msg header.
    var fixHeader = gps_msg_ as MessageWithHeader;
    SimulatorROS2Node.UpdateROSTimestamp(ref fixHeader);
    var velHeader = vel_msg_ as MessageWithHeader;
    SimulatorROS2Node.UpdateROSTimestamp(ref velHeader);

    gps_msg_.Latitude = referenceLatitude_ +
       (Mathf.Cos(referenceHeading_) * position.x + Mathf.Sin(referenceHeading_) * position.y) /
       radius_north_ * 180.0f / Mathf.PI;
    gps_msg_.Longitude = referenceLongitude_ -
      (-Mathf.Sin(referenceHeading_) * position.x + Mathf.Cos(referenceHeading_) * position.y) /
      radius_east_ * 180.0f / Mathf.PI;
    gps_msg_.Altitude = referenceAltitude_ + position.z;

    vel_msg_.Vector.X = Mathf.Cos(referenceHeading_) * velocity.x + Mathf.Sin(referenceHeading_) * velocity.y;
    vel_msg_.Vector.Y = -Mathf.Sin(referenceHeading_) * velocity.x + Mathf.Cos(referenceHeading_) * velocity.y;
    vel_msg_.Vector.Z = velocity.z;

    gps_msg_.Position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN;

    gps_msg_.Position_covariance[0] = SampleGaussian(random, 0.0f, gaussian_deviation_x);
    gps_msg_.Position_covariance[4] = SampleGaussian(random, 0.0f, gaussian_deviation_y);
    gps_msg_.Position_covariance[8] = SampleGaussian(random, 0.0f, gaussian_deviation_z);

    fixPublisher.Publish(gps_msg_);
    velPublisher.Publish(vel_msg_);
  }

  public float SampleGaussian(System.Random random, float mean, float stddev)
  {
    // The method requires sampling from a uniform random of (0,1]
    // but Random.NextDouble() returns a sample of [0,1).
    float x1 = (float)1.0f - (float)random.NextDouble();
    float x2 = (float)1.0f - (float)random.NextDouble();

    float y1 = Mathf.Sqrt(-2.0f * Mathf.Log(x1)) * Mathf.Cos(2.0f * Mathf.PI * x2);
    return y1 * stddev + mean;
  }
}
