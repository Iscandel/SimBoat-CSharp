//using System.Collections;
//using System.Collections.Generic;
//using System.Linq;
//using UnityEngine;

//public class BoatForces : MonoBehaviour, IPhysicsListener, IForceListener
//{

//    struct Triangle
//    {
//        public Triangle(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 normal, TriangleAndVertices tv)
//        {
//            this.p1 = p1;
//            this.p2 = p2;
//            this.p3 = p3;
//            this.normal = normal;
//            this.tv = tv;
//        }

//        public Vector3 p1;
//        public Vector3 p2;
//        public Vector3 p3;

//        public Vector3 normal;

//        public TriangleAndVertices tv;
//    }

//    internal struct VertexAndDepth
//    {
//        public VertexAndDepth(Vector3 vertex, float h)
//        {
//            this.vertex = vertex;
//            this.h = h;
//        }

//        public Vector3 vertex;
//        public float h;
//    }


//    struct TriangleAndVertices
//    {
//        public int triIndex;
//        public VertexAndDepth hl;
//        public VertexAndDepth hm;
//        public VertexAndDepth hh;
//    }

//    private TriangleAndVertices[] _trianglesAndVertices;
//    private Mesh _mesh;
//    List<Triangle> _submergedTriangles;
//    List<Triangle> _emergedTriangles;

//    public GameObject _underwaterGameObject;
//    Mesh _underwaterMesh;

//    float _surface = 0;
//    float _submergedSurface = 0;
//    //public float _mass = 10;

//    IWaterProvider _waterProvider;

//    Body _body;
//    BodyState _bodyState;

//    public float _mass = 100;
//    public Vector3 _cogOffset = new Vector3(0,0,0);
//    public Vector3 _diagInertia = new Vector3(50, 50, 50);

//    public RefFrame _refFrame = RefFrame.WORLD;

//    // Start is called before the first frame update
//    void Start()
//    {
//        _mesh = GetComponent<MeshFilter>().sharedMesh;
//        _trianglesAndVertices = new TriangleAndVertices[_mesh.triangles.Length / 3];
//        _submergedTriangles = new List<Triangle>();


//        Transform tr = this.transform.GetChild(0);
//        //this.transform.GetChild(0).GetComponent<GameObject>()
//        _underwaterMesh = this.transform.GetChild(0).GetComponent<MeshFilter>().mesh;//.AddComponent<GameObject>(); 

//        Vector3[] vertices = _mesh.vertices;
//        for (int i = 0; i < _trianglesAndVertices.Length; i++)
//        {
//            int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
//            Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
//            Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
//            Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);
//            Vector3 normal = Vector3.Cross(worldVertex1 - worldVertex0, worldVertex2 - worldVertex0);
//            _surface += 0.5f * normal.magnitude;
//        }

//        _waterProvider = new SimpleWaterProvider(0);
//        //_waterProvider = new SinWaterProvider();
//        //_waterProvider = new CrestWaterProvider(1);
//        //GetComponent<ChronoPhysicsManager.in

//        //
//        ChronoPhysicsManager manager = ChronoPhysicsManager.Instance;

//        _bodyState = new BodyState();


//        BodyParams bodyParams = new BodyParams();
//        bodyParams.mass = _mass;
//        bodyParams.cogOffset = _cogOffset;
//        bodyParams.diagInertia = _diagInertia;
//        //TODO init with init body state
//        _body = manager.CreateBody(bodyParams, transform.position, transform.rotation);
//        manager.addForceListener(this, _body, _refFrame);
//        manager.AddPhysicsEventListener(this);

//        //_mass = 1000;
//    }

//    // Update is called once per frame
//    void Update()
//    {

//        //ChronoPhysicsManager manager = GetComponent<ChronoPhysicsManager>();
//        //(Vector3 forces, Vector3 torques) = ComputeWeight();
//        //(Vector3 forcesB, Vector3 torquesB) = ComputeBuoyancy();

//        //forces += forcesB;

//        //manager.UpdateForces(forces, torques, Time.deltaTime);
//    }

//    void FixedUpdate()
//    {
//        //ChronoPhysicsManager manager = GetComponent<ChronoPhysicsManager>();
//        //manager.SetMass(_mass);
//        ////(Force forces, Vector3 torques) = ComputeWeight();
//        //Force forces = ComputeWeight();
//        //(Force forcesB, Vector3 torquesB) = ComputeBuoyancy();
//        //Force damping = ComputeDamping(_submergedSurface, _surface, manager.GetSpeedBody());

//        //forces.force += forcesB.force + damping.force;

//        //manager.UpdateForces(forces, Time.deltaTime);
//    }

///*    (Vector3 force, Vector3 torque) ComputeWeight()
//    {
//        Force force = new Force();
//        Vector3 torque = new Vector3();

//        Vector3 G = new Vector3(0, -9.81f, 0);
//        force.force = _mass * G;

//        //Debug.Log(force.y);

//        return (force: force, torque: torque);
//    }*/

//    Force ComputeWeight()
//    {
//        Force force = new Force();
//        Vector3 torque = new Vector3();

//        Vector3 G = new Vector3(0, -9.81f, 0);
//        force.force = _mass * G;
//        force.appliPoint = this.transform.position;

//        //Debug.Log(force.y);

//        return force;
//    }

//    Force ComputeDamping(float submergedSurface, float totalSurface, Vector3 speed_value, Vector3 cog)
//    {
//        Force force;
//        const float Cdamp = 1000;
//        float rs = submergedSurface / totalSurface;
//        force.force = -Cdamp * rs * speed_value;

//        //ok ?
//        force.appliPoint = cog;// new Vector3(0, 0, 0);

//        return force;
//    }

//    internal class SortedVector3Comparer : IComparer<VertexAndDepth>
//    {
//        public int Compare(VertexAndDepth a, VertexAndDepth b)
//        {
//            int cmp =  a.h.CompareTo(b.h);
//            if (cmp == 0)
//            {
//                if (a.vertex.x < b.vertex.x)
//                    return -1;
//                else if (a.vertex.x == b.vertex.x)
//                {
//                    if (a.vertex.z < b.vertex.z)
//                        return -1;
//                    else
//                        return 1;
//                }
//                else
//                    return 1;
//            }
//            return cmp;
//            //if(cmp == 0)
//            //    return a.vertex.x
//        }
//    }

//    //(Force force, Vector3 torque) ComputeBuoyancy()

//    Force ComputeTheoriticalBuoyancy()
//    {
//        Force force = new Force();

//        float lowY = this.transform.position.y - 0.5f;
//        if (lowY > 0)
//        {
//            force.force = new Vector3();
//            force.appliPoint = this.transform.position;
//        }
//        else
//        {
//            float volume = Mathf.Min(Mathf.Abs(lowY), 1.0f) * 1 * 1;
//            Vector3 G = new Vector3(0, 9.81f, 0);
//            float RHO = 1026;
//            force.force = RHO * volume * G;
//            force.appliPoint = this.transform.position;
//        }

//        return force;
//    }

//    List<Force> ComputeBuoyancy()
//    {
//        //float d = transform.position.y - 0.5f;

//        //Force force = new Force();
//        //Vector3 torque = new Vector3();

//        //if (d < 0)
//        //    force.y = 1026 * -System.Math.Max(d, -1) * 9.81f;

//        //Debug.Log(force.y);

//        //return (force: force, torque: torque);

//        Vector3[] vertices = _mesh.vertices;

//        _submergedTriangles.Clear();

//        Vector3[] samplingList = new Vector3[_trianglesAndVertices.Length * 3];
//        for (int i = 0; i < _trianglesAndVertices.Length; i++)
//        {
//            int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
//            Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
//            Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
//            Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

//            samplingList[i * 3] = worldVertex0;
//            samplingList[i * 3 + 1] = worldVertex1;
//            samplingList[i * 3 + 2] = worldVertex2;
//        }

//        _waterProvider.UpdateSamplingList(samplingList);

//        for (int i = 0; i < _trianglesAndVertices.Length; i++)
//        {
//            Hashtable hashtable = new Hashtable();  

//            //SortedSet<double> h = new SortedSet<double>();
//            SortedSet <VertexAndDepth> h = new SortedSet<VertexAndDepth>(new SortedVector3Comparer());
//            int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
//            Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
//            Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
//            Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

//            float waterLevel = _waterProvider.GetHeightAt(i * 3);
//            //Debug.Log(waterLevel);
//            //if (worldVertex0.y < waterLevel)
//            h.Add(new VertexAndDepth(worldVertex0, worldVertex0.y - waterLevel));

//            waterLevel = _waterProvider.GetHeightAt(i * 3 + 1);
//            //if(worldVertex1.y < waterLevel)
//            h.Add(new VertexAndDepth(worldVertex1, worldVertex1.y - waterLevel));

//            waterLevel = _waterProvider.GetHeightAt(i * 3 + 2);
//            //if(worldVertex2.y < waterLevel)
//            h.Add(new VertexAndDepth(worldVertex2, worldVertex2.y - waterLevel));

//            Vector3 currentTriNormal = (_mesh.normals[indices[0]] + _mesh.normals[indices[1]] + _mesh.normals[indices[2]]) / 3.0f;
//            //take care of scaling factor
//            currentTriNormal = transform.TransformDirection(currentTriNormal); 

//            int cpt = 0;
//            foreach(VertexAndDepth val in h)
//            {
//                if (cpt == 0)
//                    _trianglesAndVertices[i].hl = val;
//                if(cpt == 1)
//                    _trianglesAndVertices[i].hm = val;
//                if (cpt == 2)
//                    _trianglesAndVertices[i].hh = val;
//                cpt++;
//            }

         
//            // Cutting
//            if (_trianglesAndVertices[i].hh.h > 0 && _trianglesAndVertices[i].hl.h < 0)
//            {
//                //Vector3 ml = _trianglesAndVertices[i].hl.vertex - _trianglesAndVertices[i].hm.vertex;
                
//                //Vector3 
//                if (_trianglesAndVertices[i].hl.h < 0 && _trianglesAndVertices[i].hm.h < 0)
//                {
//                    Vector3 lh = _trianglesAndVertices[i].hh.vertex - _trianglesAndVertices[i].hl.vertex;
//                    Vector3 mh = _trianglesAndVertices[i].hh.vertex - _trianglesAndVertices[i].hm.vertex;

//                    float tm = -_trianglesAndVertices[i].hm.h / (_trianglesAndVertices[i].hh.h - _trianglesAndVertices[i].hm.h);
//                    Vector3 vecIm = tm * mh;
//                    Vector3 im = vecIm + _trianglesAndVertices[i].hm.vertex;

//                    float tl = -_trianglesAndVertices[i].hl.h / (_trianglesAndVertices[i].hh.h - _trianglesAndVertices[i].hl.h);
//                    Vector3 vecIl = tl * lh;
//                    Vector3 il = vecIl + _trianglesAndVertices[i].hl.vertex;

//                    Vector3 tmpNormal = Vector3.Cross(_trianglesAndVertices[i].hm.vertex - _trianglesAndVertices[i].hl.vertex, _trianglesAndVertices[i].hl.vertex - il);
//                    if (CompareNormals(tmpNormal, currentTriNormal))
//                    {
//                        _submergedTriangles.Add(new Triangle(il, _trianglesAndVertices[i].hm.vertex, _trianglesAndVertices[i].hl.vertex, currentTriNormal, _trianglesAndVertices[i]));
//                        _submergedTriangles.Add(new Triangle(im, _trianglesAndVertices[i].hm.vertex, il, currentTriNormal, _trianglesAndVertices[i]));
//                    }
//                    else
//                    {
//                        _submergedTriangles.Add(new Triangle(il, _trianglesAndVertices[i].hl.vertex, _trianglesAndVertices[i].hm.vertex, currentTriNormal, _trianglesAndVertices[i]));
//                        _submergedTriangles.Add(new Triangle(im, il, _trianglesAndVertices[i].hm.vertex, currentTriNormal, _trianglesAndVertices[i]));
//                    }

//                }
//                else if ( _trianglesAndVertices[i].hm.h > 0 && _trianglesAndVertices[i].hl.h < 0)
//                {
//                    Vector3 lm = _trianglesAndVertices[i].hm.vertex - _trianglesAndVertices[i].hl.vertex;
//                    Vector3 lh = _trianglesAndVertices[i].hh.vertex - _trianglesAndVertices[i].hl.vertex;

//                    float tm = -_trianglesAndVertices[i].hl.h / (_trianglesAndVertices[i].hm.h - _trianglesAndVertices[i].hl.h);
//                    Vector3 vecJm = tm * lm;
//                    Vector3 jm = vecJm + _trianglesAndVertices[i].hl.vertex;

//                    float th = -_trianglesAndVertices[i].hl.h / (_trianglesAndVertices[i].hh.h - _trianglesAndVertices[i].hl.h);
//                    Vector3 vecJh = th * lh;
//                    Vector3 jh = vecJh + _trianglesAndVertices[i].hl.vertex;

//                    Vector3 tmpNormal = Vector3.Cross(jm - jh, _trianglesAndVertices[i].hl.vertex - jh);
//                    if (!CompareNormals(tmpNormal, currentTriNormal))
//                        _submergedTriangles.Add(new Triangle(jh, _trianglesAndVertices[i].hl.vertex, jm, currentTriNormal, _trianglesAndVertices[i]));
//                    else
//                        _submergedTriangles.Add(new Triangle(jh, jm, _trianglesAndVertices[i].hl.vertex, currentTriNormal, _trianglesAndVertices[i]));
//                }
//            } 
//            else if(_trianglesAndVertices[i].hh.h < 0 && _trianglesAndVertices[i].hm.h < 0 && _trianglesAndVertices[i].hl.h < 0)
//                _submergedTriangles.Add(new Triangle(worldVertex0, worldVertex1, worldVertex2, currentTriNormal, _trianglesAndVertices[i]));
//        }

//        List<Vector3> uvertices = new List<Vector3>();
//        List<int> utri = new List<int>();
//        List<Color> ucolor = new List<Color>();
//        List<Vector3> unormals = new List<Vector3>();
//        Color col = Color.yellow;

//        List<Force> forces = new List<Force>();

//        _submergedSurface = 0;
//        foreach (Triangle triangle in _submergedTriangles)
//        {
//            Vector3 normal = Vector3.Cross(triangle.p2 - triangle.p1, triangle.p3 - triangle.p1);
//            float area = 0.5f * normal.magnitude;
//            _submergedSurface += area;
//            Vector3 center = ComputeCentroid(triangle.p1, triangle.p2, triangle.p3);
//            //Vector3 center = ComputeCenterOfPressure(triangle, triangle.tv);
//            Vector3 tmpForce = -1026 * 9.81f * area * triangle.normal * System.Math.Abs(center.y);
//            //Debug.Log(center.y);
//            Force force = new Force();
//            force.force = tmpForce;
//            force.appliPoint = center;
//            //forces.Add(force);


//            Force force1 = ComputeCenterOfPressure2(triangle);//, triangle.tv);
//            forces.Add(force1);
//            //force.force += transform.InverseTransformDirection(tmpForce);
//            //force.appliPoint = transform.InverseTransformPoint(center);

//            //uvertices.Add(new Vector3(0.5f, 0, -0.5f));
//            //unormals.Add(new Vector3(1.0f, 0, 0));
//            //utri.Add(uvertices.Count - 1);
//            //ucolor.Add(col);
//            //uvertices.Add(new Vector3(0.5f, 0, 0));
//            //unormals.Add(new Vector3(1.0f, 0, 0));
//            //utri.Add(uvertices.Count - 1);
//            //ucolor.Add(col);
//            //uvertices.Add(new Vector3(0.5f, -0.5f, -0.5f));
//            //unormals.Add(new Vector3(1.0f, 0, 0));
//            //utri.Add(uvertices.Count - 1);
//            //ucolor.Add(col);


//            uvertices.Add(transform.InverseTransformPoint(triangle.p1));
//            unormals.Add(transform.InverseTransformDirection(triangle.normal));
//            utri.Add(uvertices.Count - 1);
//            ucolor.Add(col);
//            uvertices.Add(transform.InverseTransformPoint(triangle.p2));
//            unormals.Add(transform.InverseTransformDirection(triangle.normal));
//            utri.Add(uvertices.Count - 1);
//            ucolor.Add(col);
//            uvertices.Add(transform.InverseTransformPoint(triangle.p3));
//            unormals.Add(transform.InverseTransformDirection(triangle.normal));
//            utri.Add(uvertices.Count - 1);
//            ucolor.Add(col);
//        }




//        _underwaterMesh.Clear();
//        _underwaterMesh.name = "underwater mesh";
//        _underwaterMesh.vertices = uvertices.ToArray();
//        _underwaterMesh.triangles = utri.ToArray();
//        _underwaterMesh.colors = ucolor.ToArray();
//        _underwaterMesh.normals = unormals.ToArray();
//        _underwaterMesh.RecalculateBounds();

//        //Debug.Log(force.y);

//        return forces;
//        //return (force: force, torque: torque);
//    }

//    Vector3 ComputeCentroid(Vector3 p1, Vector3 p2, Vector3 p3)
//    {
//        return (p1 + p2 + p3) / 3;
//    }

//    bool NeedToBeCut(TriangleAndVertices tv)
//    {
//        if (tv.hh.h == tv.hm.h || tv.hm.h == tv.hl.h)
//            return false;
//        return true;
//    }

//    //Vector3 ComputeCenterOfPressure(Triangle t, TriangleAndVertices tv)
//    //{
//    //    if(tv.hh.vertex.y == tv.hm.vertex.y || tv.hm.vertex.y == tv.hl.vertex.y)
//    //    {
//    //        //Apex is up
//    //        if (tv.hh.vertex.y > tv.hm.vertex.y)
//    //        {
//    //            double z0 = tv.hh.vertex.y;
//    //            double h = Mathf.Max(Mathf.Max(Mathf.Abs(t.p1.y - t.p2.y), Mathf.Abs(t.p1.y - t.p3.y)), Mathf.Abs(t.p2.y - t.p3.y));
//    //            double tc = (4 * z0 + 3 * h) / (6 * z0 + 4 * h);

//    //            Vector3 halfBase = (tv.hm.vertex + tv.hl.vertex) / 2;
//    //            Vector3 medianVect = halfBase - tv.hl.vertex;
//    //            medianVect.Normalize();
//    //            Vector3 center = tv.hh.vertex + (float)tc * medianVect;
//    //            return center;
//    //        }

//    //        //Apex is down
//    //        else
//    //        {
//    //            double z0 = tv.hh.vertex.y;
//    //            double h = Mathf.Max(Mathf.Max(Mathf.Abs(t.p1.y - t.p2.y), Mathf.Abs(t.p1.y - t.p3.y)), Mathf.Abs(t.p2.y - t.p3.y));
//    //            double tc = (2 * z0 + h) / (6 * z0 + 2 * h);

//    //            Vector3 halfBase = (tv.hl.vertex + tv.hm.vertex) / 2;
//    //            Vector3 medianVect = halfBase - tv.hl.vertex;
//    //            medianVect.Normalize();
//    //            Vector3 center = tv.hl.vertex + (float)tc * medianVect;
//    //            return center;
//    //        }
//    //    }
//    //    else
//    //    {
//    //        double a;
//    //        if (tv.hl.vertex.x > tv.hh.vertex.x)
//    //            a = (tv.hl.vertex.y - tv.hh.vertex.y) / (tv.hl.vertex.x - tv.hh.vertex.x);
//    //        else
//    //            a = (tv.hh.vertex.y - tv.hl.vertex.y) / (tv.hh.vertex.x - tv.hl.vertex.x);

//    //        double b = tv.hh.vertex.y - a * tv.hh.vertex.x;

//    //        Vector3 midInter = new Vector3()
//    //    }
//    //    //if (needsToBeCut(tv))
//    //    //{
//    //    //    double middlePoint = vd.hm;
//    //    //}





//    //}

//    internal class SortedVector3Comparer2 : IComparer<Vector3>
//    {
//        public int Compare(Vector3 a, Vector3 b)
//        {

//            if (a.y == b.y)
//            {
//                if (a.x < b.x)
//                    return -1;
//                else if (a.x == b.x)
//                {
//                    if (a.z < b.z)
//                        return -1;
//                    else
//                        return 1;
//                }
//                else
//                    return 1;
//            }
//            return a.y < b.y ? -1 : 1;
//        }
//    }
//    Force ComputeCenterOfPressure2(Triangle t)//, TriangleAndVertices tv)
//    {
//        float fluidDensity = 1026; // kg m^-3 
//        float gravity = -9.81f;     // m s^-1

//        SortedSet<Vector3> vertices = new SortedSet<Vector3>(new SortedVector3Comparer2());
//        vertices.Add(t.p1);
//        vertices.Add(t.p2);
//        vertices.Add(t.p3);

//        Vector3 H = vertices.ElementAt(2);// tv.hh.vertex;
//        Vector3 M = vertices.ElementAt(1);// tv.hm.vertex;
//        Vector3 L = vertices.ElementAt(0); //tv.hl.vertex;
//        Vector3 C = ComputeCentroid(H, M, L);
//        Vector3 normal = t.normal;

//        _waterProvider.UpdateSamplingList(new Vector3[] { C });
//        float depthC = _waterProvider.GetHeightAt(0) - C.y;
//        depthC = depthC < 0 ? 0 : depthC;

//        // Split the triangle into upper and lower triangles bisected by a line normal to the z-axis
//        Vector3 D = HorizontalIntercept(H, M, L);// tv);
//        Vector3 B = MidPoint(M, D);

//        // Initialise to the base midpoint (correct force calcuation for triangles with a horizontal base)    
//        float fU = 0, fL = 0;
//        Vector3 CpU = B;
//        Vector3 CpL = B;

//        // Upper triangle H > M
//        if (H.y >= M.y)
//        {
//            // Center of pressure
//            float z0 = depthC - (H.y - C.y);
//            CpU = CenterOfPressureApexUp(z0, H, M, B);

//            // Force at centroid
//            Vector3 CU = ComputeCentroid(H, M, D);
//            float hCU = depthC + (C.y - CU.y);
//            float area = ComputeTriangleArea(H, M, D);
//            fU = fluidDensity * gravity * area * hCU;
//        }

//        // Lower triangle L < M
//        if (M.y > L.y)
//        {
//            // Center of preseesure
//            float z0 = depthC + (C.y - M.y);
//            CpL = CenterOfPressureApexDn(z0, L, M, B);

//            // Force at centroid
//            Vector3 CL = ComputeCentroid(L, M, D);
//            float hCL = depthC + (C.y - CL.y);
//            float area = ComputeTriangleArea(L, M, D);
//            fL = fluidDensity * gravity * area * hCL;

//        }

//        // Calculate the force and centre of application
//        Force force;
//        force.force = normal * (fU + fL);
//        force.appliPoint = CenterOfForce(fU, fL, CpU, CpL);

//        return force;
//    }

//    Vector3 CenterOfPressureApexUp(float _z0, Vector3 _H, Vector3 _M, Vector3 _B)
//    {
//        Vector3 alt = _B - _H;
//        float h = _H.y - _M.y;
//        float tc = 2.0f / 3.0f;
//        float div = 6.0f * _z0 + 4.0f * h;
//        if (div != 0)
//        {
//            tc = (4.0f * _z0 + 3.0f * h) / div;
//        }
//        return _H + alt * tc;
//    }

//    Vector3 CenterOfPressureApexDn(float _z0, Vector3 _L, Vector3 _M, Vector3 _B)
//    {
//        Vector3 alt = _L - _B;
//        float h = _M.y - _L.y;
//        float tc = 1.0f / 3.0f;
//        float div = 6.0f * _z0 + 2.0f * h;
//        if (div != 0)
//        {
//            tc = (2.0f * _z0 + h) / div;
//        }
//        return _B + alt * tc;
//    }

//    Vector3 CenterOfForce(float _fA, float _fB, Vector3 _A, Vector3 _B)
//    {
//        float div = _fA + _fB;
//        if (div != 0)
//        {
//            float t = _fA / div;
//            return _B + (_A - _B) * t;
//        }
//        else
//        {
//            return _B;
//        }
//    }

//    Vector3 HorizontalIntercept(Vector3 H, Vector3 M, Vector3 L)//TriangleAndVertices tv)
//    {
//        // If _high.Z() = _low().Z() then _high.Z() = _low.Z() = _mid.Z()
//        // so any point on the line LH will satisfy the intercept condition,
//        // including t=0, which we set as default (and so return _low).
//        float t = 0.0f;
//        //float div = tv.hh.vertex.y - tv.hl.vertex.y;
//        float div = H.y - L.y;
//        if (Mathf.Abs(div) > Mathf.Epsilon)
//        {
//            //t = (tv.hm.vertex.y - tv.hl.vertex.y) / div;
//            t = (M.y - L.y) / div;
//        }
//        //return new Vector3(
//        //  tv.hl.vertex.x + t * (tv.hh.vertex.x - tv.hl.vertex.x),
//        //  tv.hm.vertex.y,
//        //  tv.hl.vertex.z + t * (tv.hh.vertex.z - tv.hl.vertex.z)
//        //);
//        return new Vector3(
//            L.x + t * (H.x - L.x),
//            M.y,
//            L.z + t * (H.z - L.z)
//        );
//    }

//    Vector3 MidPoint(Vector3 a, Vector3 b)
//    {
//        return (a + b) / 2.0f;
//    }

//    float ComputeTriangleArea(Vector3 p1, Vector3 p2, Vector3 p3)
//    {
//        Vector3 normal = Vector3.Cross(p2 - p1, p3 - p1);
//        float area = 0.5f * normal.magnitude;

//        return area;
//    }

//    bool CompareNormals(Vector3 a, Vector3 b)
//    {
//        if (Mathf.Sign(a.x) == Mathf.Sign(b.x) && Mathf.Sign(a.y) == Mathf.Sign(b.y) && Mathf.Sign(a.z) == Mathf.Sign(b.z))
//            return true;
//        return false;
//    }

//    private void CuttingAlgorithm()
//    {

//    }

//    public void OnPhysicsEvent(IPhysicsListener.EventType eventType)
//    {
//        if(eventType == IPhysicsListener.EventType.START)
//        {

//        }
//        if(eventType == IPhysicsListener.EventType.END)
//        {
//            ChronoPhysicsManager manager = ChronoPhysicsManager.Instance;
//            _bodyState = manager.GetBodyState(_body);

//            //float roll = 0;
//            //float pitch = 0;
//            //float yaw = 0;
//            //GetEulerAngleDegrees(rotation, ref roll, ref pitch, ref yaw);

//            Vector3 v3Pos = new Vector3(float.IsNaN(_bodyState.pos.x) ? 0.0f : _bodyState.pos.x,
//                                        float.IsNaN(_bodyState.pos.y) ? 0.0f : _bodyState.pos.y,
//                                        float.IsNaN(_bodyState.pos.z) ? 0.0f : _bodyState.pos.z);

//            //Vector3 v3EulerAngles = new Vector3(float.IsNaN(roll) ? 0.0f : roll,
//            //                                    float.IsNaN(pitch) ? 0.0f : pitch,
//            //                                    float.IsNaN(yaw) ? 0.0f : yaw);

//            //// Use GetComponent<Transform>() instead TODO
//            this.gameObject.transform.position = v3Pos;
//            this.gameObject.transform.rotation = _bodyState.rot;//.eulerAngles = v3EulerAngles;
//        }
//    }

//    public void ComputeForce(Body body, ref List<Force> force, BodyState state)
//    {
//        Vector3 sumForces = new Vector3();
//        //(Force forces, Vector3 torques) = ComputeWeight();
//        Force forces = ComputeWeight();
//        //(Force forcesB, Vector3 torquesB) = ComputeBuoyancy();
//        List<Force> forcesB = ComputeBuoyancy();
//        //Force forceTheoriB = ComputeTheoriticalBuoyancy();

//        Vector3 speed;
//        if (_refFrame == RefFrame.WORLD)
//            speed = _bodyState.speed;
//        else
//            speed = _bodyState.speed_body;
//        Force damping = ComputeDamping(_submergedSurface, _surface, speed, state.pos);
       
//        force.Add(forces);
//        foreach (Force fb in forcesB)
//        {
//            force.Add(fb);
//            sumForces += fb.force;
//        }
//        //force.Add(forceTheoriB);
//        force.Add(damping);
//        //Debug.Log(forcesB.force);
//        //forces.force += forcesB.force + damping.force;

//    }
//}
