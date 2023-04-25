using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Class for visualization of 3D field of view for camera.
    /// </summary>
    public class CameraSensorVisualization : MonoBehaviour
    {

        // ---- VARIABLES ---- //

        #region Inspector Variables

        [Header("Camera sensor to visualize")]
        [SerializeField] private Camera cameraObject = default;

        [Space(10)]
        [Tooltip("Max drawing distance of fov visualizer")]
        [Header("Parameters")]
        [SerializeField] private float maxDrawDistance = 100f;

        [Tooltip("Step size [in meter] of depth indicator drawings")]
        [SerializeField]
        [Range(1, 10)] private int depthIndicatorStepSize = 4;

        [SerializeField] 
        [Range(0.01f, 0.1f)] private float edgeThickness = 0.02f;

        [Space(10)]
        [Header("Mesh Materials")]
        [SerializeField] private Material edgeMaterial = null;
        [SerializeField] private Material fillMaterial = null;
        [SerializeField] private Material depthMaterial = null;

        #endregion

        #region Meshes

        private Mesh edgeMesh = default;
        public Mesh EdgeMesh
        {
            get => edgeMesh;
        }

        private Mesh fillMesh = default;
        public Mesh FillMesh
        {
            get => fillMesh;
        }

        private Mesh depthMesh = default;
        public Mesh DepthMesh
        {
            get => depthMesh;
        }

        #endregion

        #region Variables

        private float nearDrawDistance = 0.3f;

        private float farDrawDistance = 100f;

        private readonly int visualizationLayerID = 11;
        
        #endregion

        #region Mutables

        private float currSensorSizeX = 1f;

        private float currSensorSizeY = 1f;

        private float currFocalLength = 1f;

        private float currImageWidth = 1f;

        private float currImageHeight = 1f;

        private float currThickness = 1f;

        private bool initialized = false;

        #endregion


        // ---- METHODS ---- //

        #region Life Cycle

        private void OnEnable() 
        {
            InitVars();
            InitiliseMeshes();

            initialized = true;
        }

        private void OnDisable()
        {
            initialized = false;

            Dispose();
        }

        private void InitVars()
        {
            nearDrawDistance = cameraObject.nearClipPlane;
            farDrawDistance = cameraObject.farClipPlane > maxDrawDistance? maxDrawDistance : cameraObject.farClipPlane;

            currSensorSizeX = cameraObject.sensorSize.x;
            currSensorSizeY = cameraObject.sensorSize.y;
            currFocalLength = cameraObject.focalLength;

            currImageWidth = 1f;
            currImageHeight = 1f;

            currThickness = edgeThickness;
        }

        private void InitiliseMeshes() 
        {
            edgeMesh = new Mesh();
            edgeMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            edgeMesh.Clear();
            edgeMesh.name = "Edge";

            fillMesh = new Mesh();
            fillMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            fillMesh.Clear();
            fillMesh.name = "Fill";

            depthMesh = new Mesh();
            depthMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            depthMesh.Clear();
            depthMesh.name = "Depth";
        }
     
     
        private void Dispose()
        {
            edgeMesh.Clear();
            edgeMesh = null;

            fillMesh.Clear();
            fillMesh = null;

            depthMesh.Clear();
            depthMesh = null;
        }

        #endregion

        #region Unity Messages - Update

        private void Update() 
        {         
            if(!initialized)
            {
                return;
            }

            // check if render texture from camera sensor object is ready
            if(!IsCamereSensorValid())
            {
                return;
            }

            if(Mathf.Abs(currImageWidth - GetCameraSensorImageWidth()) > 0.001f || Mathf.Abs(currImageHeight - GetCameraSensorImageHeight()) > 0.001f)
            {
                currImageWidth = GetCameraSensorImageWidth();
                currImageHeight = GetCameraSensorImageHeight();
                UpdateMesh();
            }

            if(Mathf.Abs(currSensorSizeX - cameraObject.sensorSize.x) > 0.001f)
            {
                UpdateMesh();
                currSensorSizeX = cameraObject.sensorSize.x;
            }

            if(Mathf.Abs(currSensorSizeY - cameraObject.sensorSize.y) > 0.001f)
            {
                UpdateMesh();
                currSensorSizeY = cameraObject.sensorSize.y;
            }

            if(Mathf.Abs(currFocalLength - cameraObject.focalLength) > 0.001f)
            {
                UpdateMesh();
                currFocalLength = cameraObject.focalLength;
            }

            if(Mathf.Abs(currThickness - edgeThickness) > 0.001f)
            {
                UpdateMesh();
                currThickness = edgeThickness;
            }

            UpdateTransform();

            Graphics.DrawMesh(edgeMesh, this.transform.position, this.transform.rotation, edgeMaterial, visualizationLayerID, null, 0, null, false, false, false);
            Graphics.DrawMesh(depthMesh, this.transform.position, this.transform.rotation, depthMaterial, visualizationLayerID, null, 0, null, false, false, false);
            Graphics.DrawMesh(fillMesh, this.transform.position, this.transform.rotation, fillMaterial, visualizationLayerID, null, 0, null, false, false, false);
        }

        private void UpdateMesh()
        {
            edgeMesh.Clear();
            fillMesh.Clear();
            depthMesh.Clear();

            float vertFov = GetVerticalFOV();
            float horzFov = GetHorizontalFOV();
            
            Vector3[] nearPoints = GetFovVertices(horzFov, vertFov, nearDrawDistance);
            Vector3[] farPoints = GetFovVertices(horzFov, vertFov, farDrawDistance);
            
            Vector3[] points = GetFovVertices(horzFov, vertFov, nearDrawDistance, farDrawDistance);

            // fill mesh
            fillMesh.vertices = points;
            fillMesh.triangles = new int[] { 0, 4, 5, 5, 1, 0, 1, 5, 6, 6, 2, 1, 2, 6, 7, 7, 3, 2, 3, 7, 4, 4, 0, 3 };

            // edge mesh
            MeshInfo lineMesh = GetLineBetween(nearPoints[1], farPoints[1], edgeThickness * 0.5f); // bottom-left
            lineMesh.Join(GetLineBetween(nearPoints[2], farPoints[2], edgeThickness * 0.5f)); // top - left
            lineMesh.Join(GetLineBetween(nearPoints[3], farPoints[3], edgeThickness * 0.5f)); // top - right
            lineMesh.Join(GetLineBetween(nearPoints[4], farPoints[4], edgeThickness * 0.5f)); // bottom - right

            edgeMesh.vertices = lineMesh.Vertices;
            edgeMesh.triangles = lineMesh.Triangles;

            // depth mesh      
            MeshInfo depthLine= GetDepthIndicatorMesh(nearDrawDistance, farDrawDistance, depthIndicatorStepSize);
            depthMesh.vertices = depthLine.Vertices;
            depthMesh.triangles = depthLine.Triangles;      
        }

        private void UpdateTransform()
        {
            this.transform.position = cameraObject.gameObject.transform.position;
            this.transform.rotation = cameraObject.gameObject.transform.rotation;
        }

        #endregion

        #region Private Methods

        /// <summary>
        /// This method returns array of vertices for volume which represent fov 'cone' from zero to specific distance.
        /// </summary>
        /// <param name="horzFov">Camera horizontal fov</param>
        /// <param name="vertFov">Camera vertical fov</param>
        /// <param name="distance">Distance to where fov vertices are calculate</param>
        /// <returns>Array of vertices</returns>
        private Vector3[] GetFovVertices(float horzFov, float vertFov, float distance)
        {
            float x = distance * Mathf.Tan(horzFov * 0.5f * Mathf.Deg2Rad);
            float y = distance * Mathf.Tan(vertFov * 0.5f * Mathf.Deg2Rad);

            Vector3[] vertices = new Vector3[] {
                Vector3.zero,
                new Vector3(-x, -y, distance),
                new Vector3(-x, y, distance),
                new Vector3(x, y, distance),
                new Vector3(x, -y, distance)
            };

            return vertices;
        }

        /// <summary>
        /// This method returns array of vertices for volume which represent fov 'cone' from near to far distance.
        /// </summary>
        /// <param name="horzFov">Camera horizontal fov<</param>
        /// <param name="vertFov">Camera vertical fov</param>
        /// <param name="nearClip">Distance from where fov vertices are calculate</param>
        /// <param name="farClip">Distance to where fov vertices are calculate</param>
        /// <returns>Array of vertices</returns>
        private Vector3[] GetFovVertices(float horzFov, float vertFov, float nearClip, float farClip)
        {
            float minX = nearClip * Mathf.Tan(horzFov * 0.5f * Mathf.Deg2Rad);
            float minY = nearClip * Mathf.Tan(vertFov * 0.5f * Mathf.Deg2Rad);

            float maxX = farClip * Mathf.Tan(horzFov * 0.5f * Mathf.Deg2Rad);
            float maxY = farClip * Mathf.Tan(vertFov * 0.5f * Mathf.Deg2Rad);

            Vector3[] vertices = new Vector3[] {
                new Vector3(-minX, -minY, nearClip),
                new Vector3(-minX, minY, nearClip),
                new Vector3(minX, minY, nearClip),
                new Vector3(minX, -minY, nearClip),
                new Vector3(-maxX, -maxY, farClip),
                new Vector3(-maxX, maxY, farClip),
                new Vector3(maxX, maxY, farClip),
                new Vector3(maxX, -maxY, farClip)
            };

            return vertices;
        }

        /// <summary>
        /// This method calculate vertices and triangles for 3D line between two points
        /// </summary>
        /// <param name="startPoint">Start point of line</param>
        /// <param name="endPoint">End point of line</param>
        /// <param name="halfThickness">Half of line thickness</param>
        /// <returns>Return MesnInfo with vartices and triangles data</returns>
        private MeshInfo GetLineBetween(Vector3 startPoint, Vector3 endPoint, float halfThickness)
        {
            float length = Vector3.Distance(startPoint, endPoint);
            Vector3 direction = (endPoint - startPoint).normalized;
            Quaternion q = Quaternion.FromToRotation(Vector3.up, direction);

            Vector3[] vertices = Get3DLineAlongYDirection(halfThickness, length);

            for(int i=0; i < vertices.Length; i++)
            {
                vertices[i] = q * vertices[i];
                vertices[i] += startPoint;
            }

            MeshInfo meshInfo = new MeshInfo();
            meshInfo.Vertices = vertices;
            meshInfo.Triangles = new int[] { 0, 2, 1, 0, 3, 2, 0, 1, 4, 4, 1, 5, 5, 1, 6, 6, 1, 2, 
            2, 3, 6, 6, 3, 7, 7, 3, 0, 0, 4, 7, 7, 4, 6, 6, 4, 5};

            return meshInfo;
        }

        /// <summary>
        /// This method returns array of vertices for 3D line oriented along Y direction with specific thickness and length.
        /// </summary>
        /// <param name="halfThickness">Half thickness of line</param>
        /// <param name="length">Lenght of line</param>
        /// <returns>Array of vertices for 3D line</returns>
        private Vector3[] Get3DLineAlongYDirection(float halfThickness, float length = 1f)
        {
            Vector3[] vertices = new Vector3[8];

            vertices[0] = new Vector3(-1f, 0f, 0f) * halfThickness;
            vertices[1] = new Vector3(0f, 0f, 1f) * halfThickness;
            vertices[2] = new Vector3(1f, 0f, 0f) * halfThickness;
            vertices[3] = new Vector3(0f, 0f, -1f) * halfThickness;

            vertices[4] = Vector3.up * length + new Vector3(-1f, 0f, 0f) * halfThickness;
            vertices[5] = Vector3.up * length + new Vector3(0f, 0f, 1f) * halfThickness;
            vertices[6] = Vector3.up * length + new Vector3(1f, 0f, 0f) * halfThickness;
            vertices[7] = Vector3.up * length + new Vector3(0f, 0f, -1f) * halfThickness;

            return vertices;
        }

        /// <summary>
        /// This methods calculate mesh data for 3D depth indicator of camera fov
        /// </summary>
        /// <param name="minDistance">Distance of first indicator object</param>
        /// <param name="maxDistance">Max distance where indicator are create</param>
        /// <param name="spacing">Spacing between indicator objects</param>
        /// <returns>Return MesnInfo with vartices and triangles data</returns>
        private MeshInfo GetDepthIndicatorMesh(float minDistance, float maxDistance, float spacing)
        {
            float fovVert = GetVerticalFOV();
            float fovHorz = GetHorizontalFOV();

            MeshInfo lines = new MeshInfo();
            
            int counter = Mathf.FloorToInt( (maxDistance - minDistance) / spacing);

            // near clip plane
            Vector3[] points = GetFovVertices(fovHorz, fovVert, minDistance);     
            lines = GetLineBetween(points[1], points[2], edgeThickness * 0.5f);
            lines.Join(GetLineBetween(points[2], points[3], edgeThickness * 0.5f));
            lines.Join(GetLineBetween(points[3], points[4], edgeThickness * 0.5f));
            lines.Join(GetLineBetween(points[4], points[1], edgeThickness * 0.5f));

            if(counter > 1)
            {
                for(int i=1; i<counter+1; i++)
                {
                    points = GetFovVertices(fovHorz, fovVert, minDistance + spacing * i);
                    lines.Join(GetLineBetween(points[1], points[2], edgeThickness * 0.5f));
                    lines.Join(GetLineBetween(points[2], points[3], edgeThickness * 0.5f));
                    lines.Join(GetLineBetween(points[3], points[4], edgeThickness * 0.5f));
                    lines.Join(GetLineBetween(points[4], points[1], edgeThickness * 0.5f));
                }
            }

            return lines;
        }

        private float GetVerticalFOV()
        {
            float aspect = currImageWidth / currImageHeight;
            return Camera.HorizontalToVerticalFieldOfView(GetHorizontalFOV(), aspect);
        }

        private float GetHorizontalFOV()
        {
            float fov = Mathf.Atan2((cameraObject.sensorSize.x * 0.5f ), cameraObject.focalLength) * Mathf.Rad2Deg * 2f;
            return fov;
        }

        private bool IsCamereSensorValid()
        {
            if(cameraObject.targetTexture != null && GetCameraSensorImageWidth() > 0f && GetCameraSensorImageHeight() > 0f) 
            {
                return true;                
            }

            return false;
        }

        private float GetCameraSensorImageWidth()
        {
            if(cameraObject.targetTexture != null)
            {
                return cameraObject.targetTexture.width;
            }

            return -1f;
        }

        private float GetCameraSensorImageHeight()
        {
            if(cameraObject.targetTexture != null)
            {
                return cameraObject.targetTexture.height;
            }

            return -1f;
        }

        #endregion

        #region Helpers Classes

        /// <summary>
        /// Helper class for handling mesh data such as vertices and triangles
        /// </summary>
        public class MeshInfo
        {
            public Vector3[] Vertices = default;
            public int[] Triangles = default;

            public void Join(MeshInfo mesh)
            {
                MeshInfo meshInfo = AddMeshes(this, mesh);

                this.Vertices = meshInfo.Vertices;
                this.Triangles = meshInfo.Triangles;
            }

            public static MeshInfo AddMeshes(MeshInfo meshA, MeshInfo meshB)
            {
                MeshInfo meshInfo = new MeshInfo();

                // concatenate vertices
                meshInfo.Vertices = new Vector3[meshA.Vertices.Length + meshB.Vertices.Length];
                meshA.Vertices.CopyTo(meshInfo.Vertices, 0);
                meshB.Vertices.CopyTo(meshInfo.Vertices, meshA.Vertices.Length);

                // concatenate triangles
                meshInfo.Triangles = new int[meshA.Triangles.Length + meshB.Triangles.Length];
                meshA.Triangles.CopyTo(meshInfo.Triangles, 0);

                int[] triangle = new int[meshB.Triangles.Length];
                for(int i=0; i<meshB.Triangles.Length; i++)
                {
                    triangle[i] = meshB.Triangles[i] + meshA.Vertices.Length;
                }

                triangle.CopyTo(meshInfo.Triangles, meshA.Triangles.Length);

                return meshInfo;
            }
        }
        
        #endregion
    }

}
