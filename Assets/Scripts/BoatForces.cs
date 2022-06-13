using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoatForces : MonoBehaviour
{
    struct Triangle
    {
        public Triangle(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 normal)
        {
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
            this.normal = normal;
        }

        public Vector3 p1;
        public Vector3 p2;
        public Vector3 p3;

        public Vector3 normal;
    }

    internal struct VertexAndDepth
    {
        public VertexAndDepth(Vector3 vertex, float h)
        {
            this.vertex = vertex;
            this.h = h;
        }

        public Vector3 vertex;
        public float h;
    }


    struct TriangleAndVertices
    {
        public int triIndex;
        public VertexAndDepth hl;
        public VertexAndDepth hm;
        public VertexAndDepth hh;
    }

    private TriangleAndVertices[] _trianglesAndVertices;
    private Mesh _mesh;
    List<Triangle> _submergedTriangles;
    List<Triangle> _emergedTriangles;

    public GameObject _underwaterGameObject;
    Mesh _underwaterMesh;

    public float _mass = 10;

    // Start is called before the first frame update
    void Start()
    {
        _mesh = GetComponent<MeshFilter>().sharedMesh;
        _trianglesAndVertices = new TriangleAndVertices[_mesh.triangles.Length / 3];
        _submergedTriangles = new List<Triangle>();


        Transform tr = this.transform.GetChild(0);
        //this.transform.GetChild(0).GetComponent<GameObject>()
        _underwaterMesh = this.transform.GetChild(0).GetComponent<MeshFilter>().mesh;//.AddComponent<GameObject>(); 


        //GetComponent<ChronoPhysicsManager.in

        //_mass = 1000;
    }

    // Update is called once per frame
    void Update()
    {
        
/*        ChronoPhysicsManager manager = GetComponent<ChronoPhysicsManager>();
        (Vector3 forces, Vector3 torques) = ComputeWeight();
        (Vector3 forcesB, Vector3 torquesB) = ComputeBuoyancy();

        forces += forcesB;

        manager.UpdateForces(forces, torques, Time.deltaTime);*/
    }

    void FixedUpdate()
    {
        ChronoPhysicsManager manager = GetComponent<ChronoPhysicsManager>();
        manager.SetMass(_mass);
        (Force forces, Vector3 torques) = ComputeWeight();
        (Force forcesB, Vector3 torquesB) = ComputeBuoyancy();

        forces.force += forcesB.force;

        manager.UpdateForces(forces, torques, Time.deltaTime);
    }

    (Force force, Vector3 torque) ComputeWeight()
    {
        Force force = new Force();
        Force torque = new Vector3();

        Vector3 G = new Vector3(0, -9.81f, 0);
        force.force = _mass * G;

        Debug.Log(force.y);

        return (force: force, torque: torque);
    }

    internal class SortedVector3Comparer : IComparer<VertexAndDepth>
    {
        public int Compare(VertexAndDepth a, VertexAndDepth b)
        {
            int cmp =  a.h.CompareTo(b.h);
            if(cmp == 0) cmp = -1;
            return cmp;
            //if(cmp == 0)
            //    return a.vertex.x
        }
    }

    (Force force, Vector3 torque) ComputeBuoyancy()
    {
        //float d = transform.position.y - 0.5f;

        Force force = new Force();
        Vector3 torque = new Vector3();

        //if (d < 0)
        //    force.y = 1026 * -System.Math.Max(d, -1) * 9.81f;

        //Debug.Log(force.y);

        //return (force: force, torque: torque);

        Vector3[] vertices = _mesh.vertices;

        _submergedTriangles.Clear();

        for (int i = 0; i < _trianglesAndVertices.Length; i++)
        {
            float waterLevel = 0;
            Hashtable hashtable = new Hashtable();

           

            //SortedSet<double> h = new SortedSet<double>();
            SortedSet <VertexAndDepth> h = new SortedSet<VertexAndDepth>(new SortedVector3Comparer());
            int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
            Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
            Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
            Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

            //if (worldVertex0.y < waterLevel)
            h.Add(new VertexAndDepth(worldVertex0, worldVertex0.y - waterLevel));

            //if(worldVertex1.y < waterLevel)
                h.Add(new VertexAndDepth(worldVertex1, worldVertex1.y - waterLevel));

            //if(worldVertex2.y < waterLevel)
               h.Add(new VertexAndDepth(worldVertex2, worldVertex2.y - waterLevel));

            Vector3 currentTriNormal = (_mesh.normals[indices[0]] + _mesh.normals[indices[1]] + _mesh.normals[indices[2]]) / 3.0f;
            //take care of scaling factor
            currentTriNormal = transform.TransformDirection(currentTriNormal); 

            int cpt = 0;
            foreach(VertexAndDepth val in h)
            {
                if (cpt == 0)
                    _trianglesAndVertices[i].hl = val;
                if(cpt == 1)
                    _trianglesAndVertices[i].hm = val;
                if (cpt == 2)
                    _trianglesAndVertices[i].hh = val;
                cpt++;
            }

         
            // Cutting
            if (_trianglesAndVertices[i].hh.h > 0 && _trianglesAndVertices[i].hl.h < 0)
            {
                //Vector3 ml = _trianglesAndVertices[i].hl.vertex - _trianglesAndVertices[i].hm.vertex;
                
                //Vector3 
                if (_trianglesAndVertices[i].hl.h < 0 && _trianglesAndVertices[i].hm.h < 0)
                {
                    Vector3 lh = _trianglesAndVertices[i].hh.vertex - _trianglesAndVertices[i].hl.vertex;
                    Vector3 mh = _trianglesAndVertices[i].hh.vertex - _trianglesAndVertices[i].hm.vertex;

                    float tm = -_trianglesAndVertices[i].hm.h / (_trianglesAndVertices[i].hh.h - _trianglesAndVertices[i].hm.h);
                    Vector3 vecIm = tm * mh;
                    Vector3 im = vecIm + _trianglesAndVertices[i].hm.vertex;

                    float tl = -_trianglesAndVertices[i].hl.h / (_trianglesAndVertices[i].hh.h - _trianglesAndVertices[i].hl.h);
                    Vector3 vecIl = tl * lh;
                    Vector3 il = vecIl + _trianglesAndVertices[i].hl.vertex;
                    _submergedTriangles.Add(new Triangle(_trianglesAndVertices[i].hm.vertex, il, _trianglesAndVertices[i].hl.vertex, currentTriNormal));
                    _submergedTriangles.Add(new Triangle(il, _trianglesAndVertices[i].hm.vertex, im, currentTriNormal));
                }
                else if ( _trianglesAndVertices[i].hm.h > 0 && _trianglesAndVertices[i].hl.h < 0)
                {
                    Vector3 lm = _trianglesAndVertices[i].hm.vertex - _trianglesAndVertices[i].hl.vertex;
                    Vector3 lh = _trianglesAndVertices[i].hh.vertex - _trianglesAndVertices[i].hl.vertex;

                    float tm = -_trianglesAndVertices[i].hl.h / (_trianglesAndVertices[i].hm.h - _trianglesAndVertices[i].hl.h);
                    Vector3 vecJm = tm * lm;
                    Vector3 jm = vecJm + _trianglesAndVertices[i].hl.vertex;

                    float th = -_trianglesAndVertices[i].hl.h / (_trianglesAndVertices[i].hh.h - _trianglesAndVertices[i].hl.h);
                    Vector3 vecJh = th * lh;
                    Vector3 jh = vecJh + _trianglesAndVertices[i].hl.vertex;
                    _submergedTriangles.Add(new Triangle(_trianglesAndVertices[i].hl.vertex, jh, jm, currentTriNormal));
                }
            } 
            else if(_trianglesAndVertices[i].hh.h < 0 && _trianglesAndVertices[i].hm.h < 0 && _trianglesAndVertices[i].hl.h < 0)
                _submergedTriangles.Add(new Triangle(worldVertex0, worldVertex1, worldVertex2, currentTriNormal));
        }

        List<Vector3> uvertices = new List<Vector3>();
        List<int> utri = new List<int>();
        List<Color> ucolor = new List<Color>();
        List<Vector3> unormals = new List<Vector3>();
        Color col = Color.yellow;

        foreach (Triangle triangle in _submergedTriangles)
        {
            Vector3 normal = Vector3.Cross(triangle.p2 - triangle.p1, triangle.p3 - triangle.p1);
            float area = 0.5f * normal.magnitude;
            Vector3 center = (triangle.p1 + triangle.p2 + triangle.p3) / 3.0f;
            Vector3 tmpForce = -1026 * 9.81f * area * triangle.normal * System.Math.Abs(center.y);
            //Debug.Log(center.y);
            force.force += transform.InverseTransformDirection(tmpForce);
            force.appliPoint = transform.InverseTransformPoint(center);

            uvertices.Add(transform.InverseTransformPoint(triangle.p1));
            unormals.Add(normal);
            utri.Add(uvertices.Count - 1);
            ucolor.Add(col);
            uvertices.Add(transform.InverseTransformPoint(triangle.p2));
            unormals.Add(normal);
            utri.Add(uvertices.Count - 1);
            ucolor.Add(col);
            uvertices.Add(transform.InverseTransformPoint(triangle.p3));
            unormals.Add(normal);
            utri.Add(uvertices.Count - 1);
            ucolor.Add(col);
        }

        _underwaterMesh.Clear();
        _underwaterMesh.name = "underwater mesh";
        _underwaterMesh.vertices = uvertices.ToArray();
        _underwaterMesh.triangles = utri.ToArray();
        _underwaterMesh.colors = ucolor.ToArray();
        _underwaterMesh.normals = unormals.ToArray();
        _underwaterMesh.RecalculateBounds();

        //Debug.Log(force.y);

        return (force: force, torque: torque);
    }

    private void CuttingAlgorithm()
    {

    }
}
