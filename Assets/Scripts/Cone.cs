using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]  // 💡 This makes it run in the Editor
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class Cone : MonoBehaviour
{
    public Material material;
    public float height = 3.0f;
    public float radius = 3.0f;
    public int segments = 20;

    private Mesh mesh;
    private bool needsUpdate = false;

    private void OnValidate()
    {
        needsUpdate = true;
    }

    private void Awake()
    {
        GenerateCone();
    }

    private void Update()
    {
        if (needsUpdate)
        {
            GenerateCone();
            needsUpdate = false;
        }
    }

    void GenerateCone()
    {
        if (segments < 3) return; // minimum cone shape

        // Reuse or create mesh
        if (mesh == null)
        {
            mesh = new Mesh();
            mesh.name = "Procedural Cone";
        }

        // Assign mesh to filter
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter.sharedMesh != mesh)
            meshFilter.sharedMesh = mesh;

        // Assign material
        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
        if (material != null)
            meshRenderer.sharedMaterial = material;

        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        // Tip
        vertices.Add(new Vector3(0, height, 0)); // 0

        // Center of base
        vertices.Add(Vector3.zero); // 1

        // Circle base points
        for (int i = 0; i < segments; i++)
        {
            float angle = 2 * Mathf.PI * i / segments;
            float x = radius * Mathf.Cos(angle);
            float z = radius * Mathf.Sin(angle);
            vertices.Add(new Vector3(x, 0, z)); // 2+
        }

        // Side triangles
        for (int i = 0; i < segments; i++)
        {
            int current = i + 2;
            int next = (i + 1) % segments + 2;

            triangles.Add(0);
            triangles.Add(next);
            triangles.Add(current);
        }

        // Base triangles
        for (int i = 0; i < segments; i++)
        {
            int current = i + 2;
            int next = (i + 1) % segments + 2;

            triangles.Add(1);
            triangles.Add(current);
            triangles.Add(next);
        }

        mesh.Clear();
        mesh.SetVertices(vertices);
        mesh.SetTriangles(triangles, 0);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }
}



// using System.Collections.Generic;
// using UnityEngine;
//
// [ExecuteInEditMode]  // 💡 This makes it run in the Editor
// [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
// public class Cone : MonoBehaviour
// {
//     public Material material;
//     public float height = 3.0f;
//     public float radius = 3.0f;
//     public int segments = 20;
//
//     private Mesh mesh;
//
//     private void OnValidate()
//     {
//         GenerateCone();
//     }
//
//     private void Awake()
//     {
//         GenerateCone();
//     }
//
//     void GenerateCone()
//     {
//         if (segments < 3) return; // minimum cone shape
//
//         // Reuse or create mesh
//         if (mesh == null)
//         {
//             mesh = new Mesh();
//             mesh.name = "Procedural Cone";
//         }
//
//         // Assign mesh to filter
//         MeshFilter meshFilter = GetComponent<MeshFilter>();
//         if (meshFilter.sharedMesh != mesh)
//             meshFilter.sharedMesh = mesh;
//
//         // Assign material
//         MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
//         if (material != null)
//             meshRenderer.sharedMaterial = material;
//
//         List<Vector3> vertices = new List<Vector3>();
//         List<int> triangles = new List<int>();
//
//         // Tip
//         vertices.Add(new Vector3(0, height, 0)); // 0
//
//         // Center of base
//         vertices.Add(Vector3.zero); // 1
//
//         // Circle base points
//         for (int i = 0; i < segments; i++)
//         {
//             float angle = 2 * Mathf.PI * i / segments;
//             float x = radius * Mathf.Cos(angle);
//             float z = radius * Mathf.Sin(angle);
//             vertices.Add(new Vector3(x, 0, z)); // 2+
//         }
//
//         // Side triangles
//         for (int i = 0; i < segments; i++)
//         {
//             int current = i + 2;
//             int next = (i + 1) % segments + 2;
//
//             triangles.Add(0);
//             triangles.Add(next);
//             triangles.Add(current);
//         }
//
//         // Base triangles
//         for (int i = 0; i < segments; i++)
//         {
//             int current = i + 2;
//             int next = (i + 1) % segments + 2;
//
//             triangles.Add(1);
//             triangles.Add(current);
//             triangles.Add(next);
//         }
//
//         mesh.Clear();
//         mesh.SetVertices(vertices);
//         mesh.SetTriangles(triangles, 0);
//         mesh.RecalculateNormals();
//         mesh.RecalculateBounds();
//     }
// }
//
