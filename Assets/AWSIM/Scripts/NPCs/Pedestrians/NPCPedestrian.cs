using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// NPC pedestrian that is controlled in the scenario.
    /// </summary>
    [RequireComponent(typeof(Rigidbody), typeof(Animator))]
    public class NPCPedestrian : MonoBehaviour
    {
        [SerializeField] private new Rigidbody rigidbody;
        [SerializeField] private Transform referencePoint;
        [SerializeField, Tooltip(
             "Pedestrian Animator component.\n" +
             "The animator should have the following float parameters for proper transitions.\n" +
             "    moveSpeed: Pedestrian movement speed in m/s\n" +
             "    rotateSpeed: Pedestrian rotation speed in rad/s")]
        private Animator animator;

        [Header("Ground follow parameters")]
        [SerializeField, Tooltip("Ray-cast max distance for locating the ground.")]
        private float rayCastMaxDistance = 100f;
        [SerializeField, Tooltip("Upward offset of the ray-cast origin from the GameObject local origin for locating the ground.")]
        private float rayCastOriginOffset = 1f;

        /// <summary>
        /// Get the reference point of the pedestrian.
        /// </summary>
        public Transform ReferencePoint => referencePoint;

        private const string moveSpeedProperty = "moveSpeed";
        private const string rotateSpeedProperty = "rotateSpeed";

        private void Update()
        {
            // Switch animation based on movement speed (m/s).
            var speed2D = new Vector2(rigidbody.velocity.x, rigidbody.velocity.z).magnitude;
            animator.SetFloat(moveSpeedProperty, speed2D);

            // Switch animation based on rotation speed (rad/s).
            animator.SetFloat(rotateSpeedProperty, rigidbody.angularVelocity.magnitude);
        }

        private void Reset()
        {
            if (animator == null)
            {
                animator = GetComponent<Animator>();
            }
            if (rigidbody == null)
            {
                rigidbody = GetComponent<Rigidbody>();
            }
            if (referencePoint == null)
            {
                referencePoint = transform.Find("ReferencePoint");
            }
        }

        private void OnValidate()
        {
            rigidbody.isKinematic = true;
            rigidbody.useGravity = false;
            rigidbody.interpolation = RigidbodyInterpolation.Interpolate;
        }

        /// <summary>
        /// Move the pedestrian so that the reference point is at <paramref name="position"/>.<br/>
        /// It ray-casts downward according to the inspector settings, and if there is ground collider, the pedestrian's position is corrected to be in contact with the ground.<br/>
        /// This method should be called from FixedUpdate because <see cref="Rigidbody"/> is updated internally.
        /// </summary>
        /// <param name="position">New position of the <see cref="ReferencePoint"/>.</param>
        public void SetPosition(Vector3 position)
        {
            var rootPosition = ConvertReferenceToRoot(position);
            var groundPosition = FollowGround(rootPosition);
            rigidbody.MovePosition(groundPosition);
        }

        /// <summary>
        /// Rotate the pedestrian so that the orientation of the reference point becomes <paramref name="rotation"/>.<br/>
        /// This method should be called from FixedUpdate because <see cref="Rigidbody"/> is updated internally.
        /// </summary>
        /// <param name="rotation">New rotation of the <see cref="ReferencePoint"/>.</param>
        public void SetRotation(Quaternion rotation)
        {
            var rootRotation = ConvertReferenceToRoot(rotation);
            rigidbody.MoveRotation(rootRotation);
        }

        private Vector3 FollowGround(Vector3 position)
        {
            var origin = position + Vector3.up * rayCastOriginOffset;
            var groundLayerMask = LayerMask.GetMask(Constants.Layers.Ground);
            var groundExists = Physics.Raycast(origin, Vector3.down, out var hitInfo, rayCastMaxDistance, groundLayerMask);
            return groundExists ? hitInfo.point : position;
        }

        private Quaternion ConvertReferenceToRoot(Quaternion rotation)
        {
            return rotation * Quaternion.Inverse(referencePoint.rotation) * transform.rotation;
        }

        private Vector3 ConvertReferenceToRoot(Vector3 position)
        {
            return position - referencePoint.position + transform.position;
        }
    }
}
