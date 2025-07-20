// Stylized Water 3 by Staggart Creations (http://staggart.xyz)
// COPYRIGHT PROTECTED UNDER THE UNITY ASSET STORE EULA (https://unity.com/legal/as-terms)
//    • Copying or referencing source code for the production of new asset store, or public, content is strictly prohibited!
//    • Uploading this file to a public repository will subject it to an automated DMCA takedown request.

using UnityEngine;
using UnityEngine.Rendering;

namespace StylizedWater3.UnderwaterRendering
{
    public static class UnderwaterUtilities
    {
        private const float VERTEX_DISTANCE = 0.02f; //50 subdivisions (100 tris)
        
        private const int planeLengthSegments = 1;
        private const float SCALE = 1f; //Unit rectangle
        
        private static Mesh _WaterLineMesh;
        public static Mesh WaterLineMesh
        {
            get
            {
                if (!_WaterLineMesh)
                {
                    _WaterLineMesh = CreateMaskMesh();
                }
                
                return _WaterLineMesh;
            }
        }
        
        private static Mesh CreateMaskMesh()
        {
            int subdivisions = Mathf.FloorToInt(SCALE / VERTEX_DISTANCE);

            int xCount = subdivisions + 1;
            int yCount = planeLengthSegments + 1;
            int numTriangles = subdivisions * planeLengthSegments * 6;
            int numVertices = xCount * yCount;
    
            Vector3[] vertices = new Vector3[numVertices];
            int[] triangles = new int[numTriangles];
            Vector2[] uvs = new Vector2[numVertices];
            Color[] colors = new Color[numVertices];
    
            float scaleX = SCALE / subdivisions;
            float scaleY = SCALE / planeLengthSegments;
    
            int index = 0;
            for (int z = 0; z < yCount; z++)
            {
                for (int x = 0; x < xCount; x++)
                {
                    vertices[index] = new Vector3(x * scaleX - (SCALE * 0.5f), z * scaleY - (SCALE * 0.5f), 0f);
                    uvs[index] = new Vector2(x * scaleX, z * scaleY);
                    colors[index] = new Color(0, 0, z * scaleY, 1f);
                    index++;
                }
            }

            index = 0;
            for (int z = 0; z < planeLengthSegments; z++)
            {
                for (int x = 0; x < subdivisions; x++)
                {
                    triangles[index] = (z * xCount) + x;
                    triangles[index + 1] = ((z + 1) * xCount) + x;
                    triangles[index + 2] = (z * xCount) + x + 1;

                    triangles[index + 3] = ((z + 1) * xCount) + x;
                    triangles[index + 4] = ((z + 1) * xCount) + x + 1;
                    triangles[index + 5] = (z * xCount) + x + 1;
                    index += 6;
                }
            }

            Mesh mesh = new Mesh();

            mesh.vertices = vertices;
            mesh.uv = uvs;
            mesh.colors = colors;
            
            mesh.subMeshCount = 2;
            mesh.SetTriangles(triangles, 0); //First submesh, for underwater shading
            mesh.SetTriangles(triangles, 1); //Second submesh, waterline

            //So mesh doesn't get culled
            mesh.bounds = new Bounds(Vector3.zero, new Vector3(10000f, 10000f, 10000f));

            mesh.name = "Underwater Curtain";

            return mesh;
        }

        public static void GetNearPlaneCorners(Camera camera, float nearPlaneOffset, out Vector3 bottomLeft, out Vector3 bottomRight, out Vector3 topLeft, out Vector3 topRight)
        {
            Transform t = camera.transform;

            float z = camera.nearClipPlane + nearPlaneOffset;
            float halfHeight = camera.nearClipPlane / camera.projectionMatrix.m11;
            float halfWidth = halfHeight * camera.aspect;

            Vector3 center = t.position + t.forward * z;
            Vector3 up = t.up * halfHeight;
            Vector3 right = t.right * halfWidth;

            topLeft     = center + up - right;
            topRight    = center + up + right;
            bottomLeft  = center - up - right;
            bottomRight = center - up + right;
        }
		
		private static float GetNearPlaneHeight(Camera camera)
		{
			return camera.projectionMatrix.inverse.m11;
		}
      
        public static Vector3 GetNearPlaneBottomPosition(Camera targetCamera, float offset = 0f)
        {
            Transform transform = targetCamera.transform;
            
            return transform.position + (transform.forward * (targetCamera.nearClipPlane + offset)) - 
                   (transform.up * ((targetCamera.nearClipPlane + offset) * GetNearPlaneHeight(targetCamera)));
        }
        
        public static Vector3 GetNearPlaneTopPosition(Camera targetCamera, float offset = 0f)
        {
            Transform transform = targetCamera.transform;

            return transform.position + (transform.forward * (targetCamera.nearClipPlane + offset)) + 
                   (transform.up * ((targetCamera.nearClipPlane + offset) * GetNearPlaneHeight(targetCamera)));
        }
    }
}
