//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;

//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;

//public class BoatBuoyancyTheory : MonoBehaviour, IPhysicsListener, IForceListener
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

//    float _surface = 0;
//    float _submergedSurface = 0;
//    //public float _mass = 10;

//    IWaterProvider _waterProvider;

//    Body _body;
//    BodyStateCh _bodyState;

//    public float _mass = 100;
//    public Vector3 _cogOffset = new Vector3(0, 0, 0);
//    public Vector3 _diagInertia = new Vector3(50, 50, 50);

//    public RefFrame _refFrame = RefFrame.WORLD;

//    // Start is called before the first frame update
//    void Start()
//    {
//        _mesh = GetComponent<MeshFilter>().sharedMesh;
//        _trianglesAndVertices = new TriangleAndVertices[_mesh.triangles.Length / 3];
//        _submergedTriangles = new List<Triangle>();

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
//        //_waterProvider = new CrestWaterProvider(1);
//        //GetComponent<ChronoPhysicsManager.in

//        //
//        ChronoPhysicsManager manager = ChronoPhysicsManager.Instance;

//        _bodyState = new BodyStateCh();


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
//    }

//    void FixedUpdate()
//    {
//    }

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

//    Force ComputeDamping(float submergedSurface, float totalSurface, Vector3 speed_value)
//    {
//        Force force;
//        const float Cdamp = 1000;
//        float rs = submergedSurface / totalSurface;
//        force.force = -Cdamp * rs * speed_value;

//        //TODO
//        force.appliPoint = new Vector3(0, 0, 0);

//        return force;
//    }

//    internal class SortedVector3Comparer : IComparer<VertexAndDepth>
//    {
//        public int Compare(VertexAndDepth a, VertexAndDepth b)
//        {
//            int cmp = a.h.CompareTo(b.h);
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
//            if (lowY < -1)
//                force.appliPoint = this.transform.position;
//            else
//                force.appliPoint = new Vector3(transform.position.x, lowY / 2.0f, transform.position.z);
//        }

//        return force;
//    }

//    public void OnPhysicsEvent(IPhysicsListener.EventType eventType)
//    {
//        if (eventType == IPhysicsListener.EventType.START)
//        {

//        }
//        if (eventType == IPhysicsListener.EventType.END)
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

//    public void ComputeForce(Body body, ref List<Force> force, BodyStateCh state)
//    {
//        Vector3 sumForces = new Vector3();
//        Force forces = ComputeWeight();
//        Force forceTheoriB = ComputeTheoriticalBuoyancy();

//        Vector3 speed;
//        if (_refFrame == RefFrame.WORLD)
//            speed = _bodyState.speed;
//        else
//            speed = _bodyState.speed_body;
//        Force damping = ComputeDamping(_submergedSurface, _surface, speed);
//        force.Add(forces);

//        force.Add(forceTheoriB);

//        Debug.Log("Theory " + (forces.force + forceTheoriB.force) + " and " + state.speed + " and " + transform.position);
//        //force.Add(damping);
//        //Debug.Log(forcesB.force);
//        //forces.force += forcesB.force + damping.force;

//    }
//}

