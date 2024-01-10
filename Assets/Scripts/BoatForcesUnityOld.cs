using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;


class BoatForcesUnityOld : MonoBehaviour
{
    internal struct VertexAndDepth
    {
        public VertexAndDepth(Vector3 vertex, double h)
        {
            this.vertex = vertex;
            this.h = h;
        }

        public Vector3 vertex;
        public double h;
    }

    internal class SortedVector3Comparer : IComparer<VertexAndDepth>
    {
        public int Compare(VertexAndDepth a, VertexAndDepth b)
        {
            int cmp = a.vertex.y.CompareTo(b.vertex.y);
            if (cmp == 0)
            {
                if (a.vertex.x < b.vertex.x)
                    return -1;
                else if (a.vertex.x == b.vertex.x)
                {
                    if (a.vertex.z < b.vertex.z)
                        return -1;
                    else
                        return 1;
                }
                else
                    return 1;
            }
            return cmp;
        }
    }

    struct DepthTriangle
    {
        public DepthTriangle(Vector3 p1, Vector3 p2, Vector3 p3, double depth1, double depth2, double depth3, Vector3 normal)
        {
            vertices = new VertexAndDepth[3];
            vertices[0] = new VertexAndDepth(p1, depth1);
            vertices[1] = new VertexAndDepth(p2, depth2);
            vertices[2] = new VertexAndDepth(p3, depth3);

            //_oldIndices = new int[] { 0, 1, 2 };
            //Array.Sort(vertices, _oldIndices, new SortedVector3Comparer());

            this.normal = normal;
        }

        public (VertexAndDepth H, VertexAndDepth M, VertexAndDepth L) GetSortedByWaterHeight()
        {
            VertexAndDepth[] vert = new VertexAndDepth[3];
            vert = (VertexAndDepth[]) vertices.Clone();
            Array.Sort(vert, (a, b) => a.h.CompareTo(b.h));

            return (vert[2], vert[1], vert[0]);
        }

        public (VertexAndDepth H, VertexAndDepth M, VertexAndDepth L) GetSortedByVertex()
        {
            VertexAndDepth[] vert = new VertexAndDepth[3];
            vert = (VertexAndDepth[])vertices.Clone();
            Array.Sort(vert, (a, b) => a.vertex.y.CompareTo(b.vertex.y));

            return (vert[2], vert[1], vert[0]);
        }

        public Vector3 p0 => vertices[0].vertex;
        public Vector3 p1 => vertices[1].vertex;
        public Vector3 p2 => vertices[2].vertex;

        public Vector3 normal;

        private VertexAndDepth[] vertices;
    }

    struct Triangle
    {
        public Triangle(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 normal, TriangleAndVertices tv)
        {
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
            this.normal = normal;
            this.tv = tv;
        }

        public Vector3 p1;
        public Vector3 p2;
        public Vector3 p3;

        public Vector3 normal;

        public TriangleAndVertices tv;
    }

    struct TriangleAndVertices
    {
        public int triIndex;
        public VertexAndDepth hl;
        public VertexAndDepth hm;
        public VertexAndDepth hh;
    }

    private DepthTriangle[] _trianglesAndVertices;
    private Mesh _mesh;
    List<DepthTriangle> _submergedTriangles;
    List<DepthTriangle> _emergedTriangles;

    public GameObject _underwaterGameObject;
    Mesh _underwaterMesh;

    float _surface = 0;
    float _submergedSurface = 0;
    //public float _mass = 10;

    IWaterProvider _waterProvider;

    //Body _body;
    //BodyState _bodyState;

    public float _mass = 100;
    public Vector3 _cogOffset = new Vector3(0, 0, 0);
    public Vector3 _diagInertia = new Vector3(50, 50, 50);

    public RefFrame _refFrame = RefFrame.WORLD_UNITY;

    protected Rigidbody _rigidbody;

    public bool _computeHydrodynamics = true;



    //
    public bool _fixTimeScale = false;

    // Start is called before the first frame update
    void Start()
    {
        _mesh = GetComponent<MeshFilter>().sharedMesh;
        _trianglesAndVertices = new DepthTriangle[_mesh.triangles.Length / 3];
        _submergedTriangles = new List<DepthTriangle>();


        Transform tr = this.transform.GetChild(0);
        //this.transform.GetChild(0).GetComponent<GameObject>()
        _underwaterMesh = this.transform.GetChild(0).GetComponent<MeshFilter>().mesh;//.AddComponent<GameObject>(); 

        Vector3[] vertices = _mesh.vertices;
        for (int i = 0; i < _trianglesAndVertices.Length; i++)
        {
            int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
            Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
            Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
            Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

            _surface += ComputeTriangleArea(worldVertex0, worldVertex1, worldVertex2);// 0.5f * normal.magnitude;
        }

        //_waterProvider = new SimpleWaterProvider(0);
        _waterProvider = new SinWaterProvider();
        //_waterProvider = new CrestWaterProvider(0);

        //
        _rigidbody = GetComponent<Rigidbody>();
        _rigidbody.useGravity = false;
        _rigidbody.detectCollisions = false;
        _rigidbody.inertiaTensor = _diagInertia;
        _rigidbody.mass = _mass;
        //_rigidbody.centerOfMass = _cogOffset;
    }

    // Update is called once per frame
    void Update()
    {
        //if (_fixTimeScale)
        //{
        //    Time.timeScale = 0;
        //    _waterProvider.SetSamplingTime(Time.fixedTime);

        //    List<Force> allForces = new List<Force>();
        //    Vector3 sumForces = new Vector3();
        //    //(Force forces, Vector3 torques) = ComputeWeight();
        //    Force fweight = ComputeWeight();
        //    allForces.Add(fweight);

        //    List<Force> forcesB = ComputeBuoyancy();
        //    allForces.AddRange(forcesB);
        //}
        //else
        //    Time.timeScale = 1;
    }

    void FixedUpdate()
    {       
        //_waterProvider.SetSamplingTime(Time.fixedTime);

        List<Force> allForces = new List<Force>();
        Vector3 sumForces = new Vector3();
        //(Force forces, Vector3 torques) = ComputeWeight();
        Force fweight = ComputeWeight();
        allForces.Add(fweight);

        List<Force> forcesB = ComputeBuoyancy();
        allForces.AddRange(forcesB);

        //Force forceTheoriB = ComputeTheoriticalBuoyancy();
        //allForces.Add(forceTheoriB);



        //allForces.Add(damping);
        
        if (_computeHydrodynamics)    
        {
            allForces.AddRange(ComputeEmpiricalDrag());
            //allForces.AddRange(ComputeHydrodynamics());
        }

        foreach (Force fb in allForces)
        {
            if (_refFrame == RefFrame.WORLD_UNITY)
                _rigidbody.AddForceAtPosition(fb.force, fb.appliPoint);
            else
                _rigidbody.AddForceAtPosition(transform.InverseTransformDirection(fb.force), transform.InverseTransformPoint(fb.appliPoint));
            sumForces += fb.force;
        }




        Debug.Log("unity " + sumForces + " and " + _rigidbody.velocity + " and " + transform.position);

        (Vector3 damping2, Vector3 torque) = ComputeDamping2(_submergedSurface, _surface, _rigidbody.velocity, _rigidbody.angularVelocity, transform.position);
        //(Vector3 damping2, Vector3 torque) = ComputeDamping3(_rigidbody.velocity, _rigidbody.angularVelocity);
        //_rigidbody.AddForce(damping2);
        //_rigidbody.AddTorque(torque);

        //Vector3 speed;
        //if (_refFrame == RefFrame.WORLD)
        //    speed = _bodyState.speed;
        //else
        //    speed = _bodyState.speed_body;
        //Force damping = ComputeDamping(_submergedSurface, _surface, speed);
        //force.Add(forces);

        //force.Add(forceTheoriB);
        //force.Add(damping);
        //Debug.Log(forcesB.force);
        //forces.force += forcesB.force + damping.force;
    }

    /*    (Vector3 force, Vector3 torque) ComputeWeight()
        {
            Force force = new Force();
            Vector3 torque = new Vector3();

            Vector3 G = new Vector3(0, -9.81f, 0);
            force.force = _mass * G;

            //Debug.Log(force.y);

            return (force: force, torque: torque);
        }*/

    Force ComputeWeight()
    {
        Force force = new Force();
        Vector3 torque = new Vector3();

        force.force = _mass * UnityPhysicsConstants.G;
        force.appliPoint = this.transform.position;

        //Debug.Log(force.y);

        return force;
    }

    Force ComputeDamping(float submergedSurface, float totalSurface, Vector3 speed_value, Vector3 cog)
    {
        Force force;
        const float Cdamp = 1000;
        float rs = submergedSurface / totalSurface;
        force.force = -Cdamp * rs * speed_value;

        //Ok ?
        force.appliPoint = cog;

        return force;
    }

    (Vector3, Vector3) ComputeDampingBis(float submergedSurface, float totalSurface, Vector3 speed_value, Vector3 angularSpeed)
    {
        const float Cdamp = 1000;
        float rs = submergedSurface / totalSurface;
        Vector3 force = -Cdamp * rs * speed_value;

        //Ok ?
        Vector3 torque = -Cdamp * 0.0001f * rs * angularSpeed;

        return (force, torque);
    }

    public (Vector3, Vector3) ComputeDamping2(float submergedSurface, float totalSurface, Vector3 speed_value, Vector3 angularSpeed, Vector3 cog)
    {
        // Linear drag coefficients
        float cDampL1 = 1000f;
        float cDampR1 = 100f;

        // Quadratic drag coefficients
        float cDampL2 = 100f;
        float cDampR2 = 10f;

        float area = totalSurface;
        float subArea = submergedSurface;
        float rs = subArea / area;

        // Force
        float linSpeed = speed_value.magnitude;
        float cL = -rs * (cDampL1 + cDampL2 * linSpeed);
        Vector3 force = speed_value * cL;

        float angSpeed = angularSpeed.magnitude;
        float cR = -rs * (cDampR1 + cDampR2 * angSpeed);
        Vector3 torque = angularSpeed * cR;
        return (force, torque);
    }

    public (Vector3, Vector3) ComputeDamping3(Vector3 speed_body, Vector3 angularSpeed)
    {
        Vector3 force;
        float RHO = UnityPhysicsConstants.RHO;
        float referenceArea = _submergedSurface / 4;
        float Cdx = 1.05f * 10;
        float Cdy = 1.5f * 10;
        float Cdz = 0.5f * 10;
        force.x = -0.5f * RHO * referenceArea * Cdx * Mathf.Abs(speed_body.x) * speed_body.x;
        force.y = -0.5f * RHO * referenceArea * Cdy * Mathf.Abs(speed_body.y) * speed_body.y;
        force.z = -0.5f * RHO * referenceArea * Cdz * Mathf.Abs(speed_body.z) * speed_body.z;

        Vector3 torque;
        torque.x = -0.5f * RHO * referenceArea * Cdx / 5 * Mathf.Abs(angularSpeed.x) * angularSpeed.x;
        torque.y = -0.5f * RHO * referenceArea * Cdy / 5 * Mathf.Abs(angularSpeed.y) * angularSpeed.y;
        torque.z = -0.5f * RHO * referenceArea * Cdz / 5 * Mathf.Abs(angularSpeed.z) * angularSpeed.z;

        return (force, torque);
    }

    Force ComputeViscousDrag(Vector3 speedAtTriangle, float area, Vector3 normal, Vector3 center)
    {
        Force force;

        float viscousDragCoeff = 1.0f;
        float cosTheta = Vector3.Dot(speedAtTriangle.normalized, normal);
        if (cosTheta >= 0)
            force.force = -(1 - cosTheta) * viscousDragCoeff * speedAtTriangle * area;
        else
            force.force = (1 - cosTheta) * viscousDragCoeff * speedAtTriangle * area;
        force.appliPoint = center;
        return force;

        float RHO = UnityPhysicsConstants.RHO;

        double viscosity = 1.0533E-6f;
        double L = 2.5 * 2 + 1 * 2; //TODO
        double speedMagnitude = speedAtTriangle.magnitude;
        double Rn = speedMagnitude * L / viscosity;
        double Cf = 0.075 / Math.Pow(Math.Log10(Rn) - 2, 2);
        //Vector3 vi = speedAtTriangle - normal * Vector3.Dot(speedAtTriangle.normalized, normal);
        Vector3 vti = speedAtTriangle - normal * Vector3.Dot(speedAtTriangle.normalized, normal);
        Vector3 ufi = -vti.normalized;
        Vector3 vfi = (float)speedMagnitude * ufi;
        Vector3 drag = 0.5f * RHO * (float)Cf * area * vfi.magnitude * vfi;

        force.force = drag;
        force.appliPoint = center;

        return force;
    }

    Force ComputePressureDrag(Vector3 speedAtTriangle, float area, Vector3 normal, Vector3 center)
    {
        Force force;
        Vector3 Fd;
        float Cpd1 = 10000;
        float Cpd2 = 10000;
        float vi = speedAtTriangle.magnitude;
        float vr = 1;
        float fp = 0.4f;
        float cosTheta = Vector3.Dot(speedAtTriangle.normalized, normal);
        float Csd1 = 10000;
        float Csd2 = 10000;
        float fs = 0.4f;
        if (cosTheta >= 0)
        {
            Fd = -(Cpd1 * vi / vr + Cpd2 * (float)Math.Pow(vi / vr, 2)) * area * (float)Math.Pow(cosTheta, fp) * normal;
        }
        else
        {
            Fd = (Csd1 * vi / vr + Csd2 * (float)Math.Pow(vi / vr, 2)) * area * (float)Math.Pow(-cosTheta, fs) * normal;
        }

        force.force = Fd;
        force.appliPoint = center;

        return force;
    }

    List<Force> ComputeEmpiricalDrag()
    {
        List<Force> forces = new List<Force>();

        foreach (DepthTriangle triangle in _submergedTriangles)
        {
            float RHO = UnityPhysicsConstants.RHO;
            float area = ComputeTriangleArea(triangle.p0, triangle.p1, triangle.p2);
            Vector3 center = ComputeCentroid(triangle.p0, triangle.p1, triangle.p2);

            //float depthCenter = _waterProvider.GetHeightAt(center) - center.y;
            //if (depthCenter < 0)
            //    continue;

            //Test drag
            Vector3 p = center - _rigidbody.worldCenterOfMass;
            float VelocityDotPower = 1;
            float DynamicForcePower = 1;
            Vector3 dynamicForce;
            Vector3 velocity = _rigidbody.velocity + Vector3.Cross(_rigidbody.angularVelocity, p);
            float velocityMagnitude = velocity.magnitude;
            float DynamicForceFactor = 1;
            float dot = Vector3.Dot(triangle.normal, velocity.normalized);
            float densityArea = RHO * area;
            float SkinDrag = 1;
            {
                // Dynamic force
                if (dot < -0.0001f || dot > 0.0001f)
                {
                    dot = Mathf.Sign(dot) * Mathf.Pow(Mathf.Abs(dot), VelocityDotPower);
                }

                if (DynamicForcePower < 0.9999f || DynamicForcePower > 1.0001f)
                {
                    dynamicForce = -dot * Mathf.Pow(velocityMagnitude, DynamicForcePower * 1.0f) //ObjDynamicForcePowerCoeffs[objIndex])
                                        * densityArea * DynamicForceFactor * 1.0f * triangle.normal; //ObjDynamicForceCoeffs[objIndex] * normal;
                }
                else
                {
                    dynamicForce = -dot * velocityMagnitude * densityArea * DynamicForceFactor * 1.0f * triangle.normal;// ObjDynamicForceCoeffs[objIndex] * normal;
                }
            }

            {
                float absDot = dot < 0 ? -dot : dot;
                dynamicForce += (1f - absDot) * SkinDrag * 1.0f * densityArea * -velocity;// ObjSkinFrictionDragCoeffs[objIndex] * densityArea * -velocity;
            }

            Force drag;
            drag.force = dynamicForce;
            drag.appliPoint = center;
            forces.Add(drag);
        }

        return forces;
    }

    List<Force> ComputeHydrodynamics()
    {
        List<Force> forces = new List<Force>();

        foreach (DepthTriangle triangle in _submergedTriangles)
        {
            float area = ComputeTriangleArea(triangle.p0, triangle.p1, triangle.p2);
            Vector3 center = ComputeCentroid(triangle.p0, triangle.p1, triangle.p2);
            Vector3 p = center - _rigidbody.worldCenterOfMass;
            Vector3 speedAtTriangle = _rigidbody.velocity + Vector3.Cross(_rigidbody.angularVelocity, p);
            forces.Add(ComputeViscousDrag(speedAtTriangle, area, triangle.normal, center));
            forces.Add(ComputePressureDrag(speedAtTriangle, area, triangle.normal, center));
        }



        //Vector3 speed;
        //if (_refFrame == RefFrame.WORLD)
        //    speed = _rigidbody.velocity;
        //else
        //    speed = transform.InverseTransformDirection(_rigidbody.velocity);
        //Force damping = ComputeDamping(_submergedSurface, _surface, speed, transform.position);
        //(Vector3 dbis, Vector3 dtbis) = ComputeDampingBis(_submergedSurface, _surface, speed, _rigidbody.angularVelocity);
        ////_rigidbody.AddForce(dbis);
        ////_rigidbody.AddTorque(dtbis);

        return forces;
    }

    //(Force force, Vector3 torque) ComputeBuoyancy()

    Force ComputeTheoriticalBuoyancy()
    {
        Force force = new Force();

        float lowY = this.transform.position.y - 0.5f;
        if (lowY > 0)
        {
            force.force = new Vector3();
            force.appliPoint = this.transform.position;
        }
        else
        {
            float volume = Mathf.Min(Mathf.Abs(lowY), 1.0f) * 1 * 1;
            Vector3 G = -UnityPhysicsConstants.G;
            float RHO = UnityPhysicsConstants.RHO;
            force.force = RHO * volume * G;
            if (lowY < -1)
                force.appliPoint = this.transform.position;
            else
                force.appliPoint = new Vector3(transform.position.x, lowY / 2.0f, transform.position.z);
        }

        return force;
    }



    List<Force> ComputeBuoyancy()
    {
        Vector3[] vertices = _mesh.vertices;

        _submergedTriangles.Clear();

        Vector3[] samplingList = new Vector3[_trianglesAndVertices.Length * 3];
        for (int i = 0; i < _trianglesAndVertices.Length; i++)
        {
            int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
            Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
            Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
            Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

            samplingList[i * 3] = worldVertex0;
            samplingList[i * 3 + 1] = worldVertex1;
            samplingList[i * 3 + 2] = worldVertex2;
        }

        float[] verticesHeight = new float[samplingList.Length];
        if (!_waterProvider.SampleHeightAt(samplingList, ref verticesHeight))
        {
            Debug.LogError("Sampling error");
            return new List<Force>();
        }

        for (int i = 0; i < _trianglesAndVertices.Length; i++)
        {
            int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
            Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
            Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
            Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

            //See if we keep
            //Vector3[] samplingList = new Vector3[_trianglesAndVertices.Length * 3];
            //Vector3[] samplingList = new Vector3[3];
            //samplingList[0] = worldVertex0;
            //samplingList[1] = worldVertex1;
            //samplingList[2] = worldVertex2;
            //_waterProvider.UpdateSamplingList(samplingList);
            //

            // Height relative to surface water. Negative values mean underwater vertices
            float height0 = worldVertex0.y - verticesHeight[i*3    ];
            float height1 = worldVertex1.y - verticesHeight[i*3 + 1];
            float height2 = worldVertex2.y - verticesHeight[i*3 + 2];
            //float height0 = worldVertex0.y - _waterProvider.GetHeightAt(i*3); 
            //float height1 = worldVertex1.y - _waterProvider.GetHeightAt(i*3+1);
            //float height2 = worldVertex2.y - _waterProvider.GetHeightAt(i*3+2);

            Vector3 currentTriNormal = (_mesh.normals[indices[0]] + _mesh.normals[indices[1]] + _mesh.normals[indices[2]]) / 3.0f;
            //should take care of the scaling factor...
            currentTriNormal = transform.TransformDirection(currentTriNormal);
            currentTriNormal.Normalize();

            DepthTriangle triangle = new DepthTriangle(worldVertex0, worldVertex1, worldVertex2,
                                                       height0, height1, height2,
                                                       currentTriNormal);
            _trianglesAndVertices[i] = triangle;

            (VertexAndDepth H, VertexAndDepth M, VertexAndDepth L) = triangle.GetSortedByWaterHeight();

            Debug.Assert(H.h >= M.h && H.h >= L.h && M.h >= L.h);
            // Cutting triangles
            if (H.h > 0 && L.h < 0)
            {
                //Vector3 ml = _trianglesAndVertices[i].hl.vertex - _trianglesAndVertices[i].hm.vertex;

                // 2 underwater
                if (L.h < 0 && M.h < 0)
                {
                    Vector3 lh = H.vertex - L.vertex;
                    Vector3 mh = H.vertex - M.vertex;

                    float tm = (float)(-M.h / (H.h - M.h));
                    Vector3 vecIm = tm * mh;
                    Vector3 im = vecIm + M.vertex;

                    float tl = (float)(-L.h / (H.h - L.h));
                    Vector3 vecIl = tl * lh;
                    Vector3 il = vecIl + L.vertex;

                    Vector3 tmpNormal = Vector3.Cross(M.vertex - L.vertex, L.vertex - il);

                    //_waterProvider.UpdateSamplingList(new Vector3[] { il, im });
                    float depthIl = 0;// il.y - _waterProvider.GetHeightAt(0);
                    float depthIm = 0;// im.y - _waterProvider.GetHeightAt(1);
                    if (CompareNormals(tmpNormal, currentTriNormal))
                    {
                        _submergedTriangles.Add(new DepthTriangle(il, M.vertex, L.vertex, depthIl, M.h, L.h, currentTriNormal));
                        _submergedTriangles.Add(new DepthTriangle(im, M.vertex, il, depthIm, M.h, depthIl, currentTriNormal));
                    }
                    else
                    {
                        _submergedTriangles.Add(new DepthTriangle(il, L.vertex, M.vertex, depthIl, L.h, M.h, currentTriNormal));
                        _submergedTriangles.Add(new DepthTriangle(im, il, M.vertex, depthIm, depthIl, M.h, currentTriNormal));
                    }

                } //1 underwater
                else if (M.h > 0 && L.h < 0)
                {
                    Vector3 lm = M.vertex - L.vertex;
                    Vector3 lh = H.vertex - L.vertex;

                    float tm = (float)(-L.h / (M.h - L.h));
                    Vector3 vecJm = tm * lm;
                    Vector3 jm = vecJm + L.vertex;

                    float th = (float)(-L.h / (H.h - L.h));
                    Vector3 vecJh = th * lh;
                    Vector3 jh = vecJh + L.vertex;

                    Vector3 tmpNormal = Vector3.Cross(jm - jh, L.vertex - jh);

                    //_waterProvider.UpdateSamplingList(new Vector3[] { jm, jh });
                    float depthJm = 0;// jm.y - _waterProvider.GetHeightAt(0);
                    float depthJh = 0; // jh.y - _waterProvider.GetHeightAt(1);


                    if (!CompareNormals(tmpNormal, currentTriNormal))
                        _submergedTriangles.Add(new DepthTriangle(jh, L.vertex, jm, depthJh, L.h, depthJm, currentTriNormal));
                    else
                        _submergedTriangles.Add(new DepthTriangle(jh, jm, L.vertex, depthJh, depthJm, L.h, currentTriNormal));
                }
            }
            else if (H.h < 0 && M.h < 0 && L.h < 0)
                _submergedTriangles.Add(new DepthTriangle(worldVertex0, worldVertex1, worldVertex2, height0, height1, height2, currentTriNormal));
        }

        List<Vector3> uvertices = new List<Vector3>();
        List<int> utri = new List<int>();
        List<Color> ucolor = new List<Color>();
        List<Vector3> unormals = new List<Vector3>();
        Color col = Color.yellow;

        List<Force> forces = new List<Force>();

        _submergedSurface = 0;
        foreach (DepthTriangle triangle in _submergedTriangles)
        {
            (VertexAndDepth H, VertexAndDepth M, VertexAndDepth L) = triangle.GetSortedByVertex();
            Debug.Assert(H.vertex.y >= M.vertex.y && H.vertex.y >= L.vertex.y && M.vertex.y >= L.vertex.y);

            float RHO = UnityPhysicsConstants.RHO;
            float absG = -UnityPhysicsConstants.G.y;
            float area = ComputeTriangleArea(H.vertex, M.vertex, L.vertex);
            //Vector3 normal = Vector3.Cross(triangle.H - triangle.L, triangle.M - triangle.L);
            //float area = 0.5f * normal.magnitude;
            _submergedSurface += area;
            Vector3 center = ComputeCentroid(H.vertex, M.vertex, L.vertex);
            //Debug.Log(center.y);

            float[] centerHeight = new float[1];
            _waterProvider.SampleHeightAt(new Vector3[] {center}, ref centerHeight);
            float depthCenter = centerHeight[0] - center.y;

            //float depthCenter = _waterProvider.GetHeightAt(center) - center.y;
            if (depthCenter < 0)
            {
                Debug.Log("Depth center : " + depthCenter);
                continue;
            }

            //float distanceToSurface = 0;
            //if (area > 1e-7)
            //{
            //    Vector3 f0 = H.vertex - center;
            //    Vector3 f1 = M.vertex - center;
            //    Vector3 f2 = L.vertex - center;
            //    float doubleArea = area * 2f;
            //    float w0 = Vector3.Cross(f1, f2).magnitude / doubleArea;
            //    float w1 = Vector3.Cross(f2, f0).magnitude / doubleArea;
            //    float w2 = 1f - (w0 + w1);
            //    Debug.Assert(w0 + w1 + w2 > 0.95f && w0 + w1 + w2 < 1.05f);

            //    //if (SimulateWaterNormals)
            //    //{
            //    //    Vector3 n0 = WaterNormals[_vertIndex0];
            //    //    Vector3 n1 = WaterNormals[_vertIndex1];
            //    //    Vector3 n2 = WaterNormals[_vertIndex2];

            //    //    distanceToSurface =
            //    //        w0 * dist0 * Vector3.Dot(n0, GlobalUpVector) +
            //    //        w1 * dist1 * Vector3.Dot(n1, GlobalUpVector) +
            //    //        w2 * dist2 * Vector3.Dot(n2, GlobalUpVector);

            //    //    waterNormalVector = w0 * n0 + w1 * n1 + w2 * n2;
            //    //}
            //    //else
            //    {
            //        distanceToSurface =
            //            w0 * (-(float)H.h) +
            //            w1 * (-(float)M.h) +
            //            w2 * (-(float)L.h);
            //    }
            //    Force force;
            //    force.force = distanceToSurface * RHO * area * (-9.81f) * Vector3.up *
            //               Vector3.Dot(triangle.normal, Vector3.up);
            //    force.appliPoint = center;
            //    forces.Add(force);
            //}
            //Force force = ComputeBuoyancyAtCentroid(H.vertex, M.vertex, L.vertex, triangle.normal, center, depthCenter);      
            //forces.Add(force);


            //Force force1 = ComputeCenterOfPressure2(triangle.H, triangle.M, triangle.L, triangle.normal);
            //force1.force.x = force.force.z = 0;
            //forces.Add(force1);

            Force force = ComputeBuoyancyAtCenterOfPressure(H.vertex, M.vertex, L.vertex, triangle.normal);
            //force.force.x = force.force.z = 0;
            forces.Add(force);

            //Debug.DrawLine(force.appliPoint, force.appliPoint + triangle.normal.normalized * 3, Color.magenta);




            //force.force += transform.InverseTransformDirection(tmpForce);
            //force.appliPoint = transform.InverseTransformPoint(center);

            uvertices.Add(transform.InverseTransformPoint(triangle.p0));
            unormals.Add(transform.InverseTransformDirection(triangle.normal));
            utri.Add(uvertices.Count - 1);
            ucolor.Add(col);
            uvertices.Add(transform.InverseTransformPoint(triangle.p1));
            unormals.Add(transform.InverseTransformDirection(triangle.normal));
            utri.Add(uvertices.Count - 1);
            ucolor.Add(col);
            uvertices.Add(transform.InverseTransformPoint(triangle.p2));
            unormals.Add(transform.InverseTransformDirection(triangle.normal));
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

        //Debug.Log("UNITY " + _underwaterMesh.vertices.Length);

        //Debug.Log(force.y);

        return forces;
        //return (force: force, torque: torque);
    }

    Vector3 ComputeCentroid(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        return (p1 + p2 + p3) / 3;
    }

    bool NeedToBeCut(TriangleAndVertices tv)
    {
        if (tv.hh.h == tv.hm.h || tv.hm.h == tv.hl.h)
            return false;
        return true;
    }

    Force ComputeBuoyancyAtCentroid(Vector3 H, Vector3 M, Vector3 L, Vector3 normal, Vector3 centroid, float depthCentroid)
    {
        Force force;
        float RHO = UnityPhysicsConstants.RHO;
        float absG = Mathf.Abs(UnityPhysicsConstants.G.y);

        float area = ComputeTriangleArea(H, M, L);

        Vector3 tmpForce = -RHO * absG * area * normal * System.Math.Abs(depthCentroid);
        //Debug.Log(center.y);

        force.force = tmpForce;
        force.appliPoint = centroid;

        return force;
    }

    internal class SortedVector3Comparer2 : IComparer<Vector3>
    {
        public int Compare(Vector3 a, Vector3 b)
        {

            if (a.y == b.y)
            {
                if (a.x < b.x)
                    return -1;
                else if (a.x == b.x)
                {
                    if (a.z < b.z)
                        return -1;
                    else
                        return 1;
                }
                else
                    return 1;
            }
            return a.y < b.y ? -1 : 1;
        }
    }
    Force ComputeCenterOfPressure2(Vector3 H, Vector3 M, Vector3 L, Vector3 normal)
    {
        float fluidDensity = UnityPhysicsConstants.RHO; // kg m^-3 
        float gravity = UnityPhysicsConstants.G.y;     // m s^-1

        Vector3 C = ComputeCentroid(H, M, L);

        //_waterProvider.UpdateSamplingList(new Vector3[] { C });
        //float depthC = _waterProvider.GetHeightAt(0) - C.y;


        float[] heightC = new float[1];
        _waterProvider.SampleHeightAt(new Vector3[] { C }, ref heightC);
        float depthC = heightC[0] - C.y;
        depthC = depthC < 0 ? 0 : depthC;

        // Split the triangle into upper and lower triangles bisected by a line normal to the z-axis
        Vector3 D = HorizontalIntercept(H, M, L);// tv);
        Vector3 B = MidPoint(M, D);

        // Initialise to the base midpoint (correct force calcuation for triangles with a horizontal base)    
        float fU = 0, fL = 0;
        Vector3 CpU = B;
        Vector3 CpL = B;

        // Upper triangle H > M
        if (H.y >= M.y)
        {
            // Center of pressure
            float z0 = depthC - (H.y - C.y);
            CpU = CenterOfPressureApexUp(z0, H, M, B);

            // Force at centroid
            Vector3 CU = ComputeCentroid(H, M, D);
            float hCU = depthC + (C.y - CU.y);
            float area = ComputeTriangleArea(H, M, D);
            fU = fluidDensity * gravity * area * hCU;
        }

        // Lower triangle L < M
        if (M.y > L.y)
        {
            // Center of preseesure
            float z0 = depthC + (C.y - M.y);
            CpL = CenterOfPressureApexDn(z0, L, M, B);

            // Force at centroid
            Vector3 CL = ComputeCentroid(L, M, D);
            float hCL = depthC + (C.y - CL.y);
            float area = ComputeTriangleArea(L, M, D);
            fL = fluidDensity * gravity * area * hCL;

        }

        // Calculate the force and centre of application
        Force force;
        force.force = normal * (fU + fL);
        force.appliPoint = CenterOfForce(fU, fL, CpU, CpL);

        return force;
    }

    Vector3 CenterOfPressureApexUp(float _z0, Vector3 _H, Vector3 _M, Vector3 _B)
    {
        Vector3 alt = _B - _H;
        float h = _H.y - _M.y;
        float tc = 2.0f / 3.0f;
        float div = 6.0f * _z0 + 4.0f * h;
        if (div != 0)
        {
            tc = (4.0f * _z0 + 3.0f * h) / div;
        }
        return _H + alt * tc;
    }

    Vector3 CenterOfPressureApexDn(float _z0, Vector3 _L, Vector3 _M, Vector3 _B)
    {
        Vector3 alt = _L - _B;
        float h = _M.y - _L.y;
        float tc = 1.0f / 3.0f;
        float div = 6.0f * _z0 + 2.0f * h;
        if (div != 0)
        {
            tc = (2.0f * _z0 + h) / div;
        }
        return _B + alt * tc;
    }

    Vector3 CenterOfForce(float _fA, float _fB, Vector3 _A, Vector3 _B)
    {
        float div = _fA + _fB;
        if (div != 0)
        {
            float t = _fA / div;
            return _B + (_A - _B) * t;
        }
        else
        {
            return _B;
        }
    }

    Vector3 HorizontalIntercept(Vector3 H, Vector3 M, Vector3 L)//TriangleAndVertices tv)
    {
        // If _high.Z() = _low().Z() then _high.Z() = _low.Z() = _mid.Z()
        // so any point on the line LH will satisfy the intercept condition,
        // including t=0, which we set as default (and so return _low).
        float t = 0.0f;
        //float div = tv.hh.vertex.y - tv.hl.vertex.y;
        float div = H.y - L.y;
        if (Mathf.Abs(div) > Mathf.Epsilon)
        {
            //t = (tv.hm.vertex.y - tv.hl.vertex.y) / div;
            t = (M.y - L.y) / div;
        }
        //return new Vector3(
        //  tv.hl.vertex.x + t * (tv.hh.vertex.x - tv.hl.vertex.x),
        //  tv.hm.vertex.y,
        //  tv.hl.vertex.z + t * (tv.hh.vertex.z - tv.hl.vertex.z)
        //);
        return new Vector3(
            L.x + t * (H.x - L.x),
            M.y,
            L.z + t * (H.z - L.z)
);
    }

    Vector3 MidPoint(Vector3 a, Vector3 b)
    {
        return (a + b) / 2.0f;
    }



    Force ComputeBuoyancyAtCenterOfPressure(Vector3 H, Vector3 M, Vector3 L, Vector3 normal)
    {
        float RHO = UnityPhysicsConstants.RHO;
        float g = -UnityPhysicsConstants.G.y;

        Vector3 P = HorizontalIntersection(H, M, L);
        Vector3 medianPoint = MidPoint(P, M);

        float fUp = 0;
        float fDown = 0;
        Vector3 cpUp = new Vector3(0, 0, 0);
        Vector3 cpDown = new Vector3(0, 0, 0);

        //
        //Vector3 centroid = ComputeCentroid(H, M, L);
        //float depthC = _waterProvider.GetHeightAt(centroid) - centroid.y;

        // If upper base is not horizontal, compute apex up part
        // Use >= to handle horizontal triangles
        if (H.y >= M.y)
        {
            float[] waterAtH = new float[1];
            _waterProvider.SampleHeightAt(new Vector3[] { H }, ref waterAtH);
            float z0 = waterAtH[0] - H.y;

            //float z0 = _waterProvider.GetHeightAt(H) - H.y;
            Vector3 centroidUp = ComputeCentroid(H, M, P);
            float[] waterAtC = new float[1];
            _waterProvider.SampleHeightAt(new Vector3[] { centroidUp }, ref waterAtC);
            float depthCentroidUp = waterAtC[0] - centroidUp.y;
            //float depthCentroidUp = _waterProvider.GetHeightAt(centroidUp) - centroidUp.y;
            if (depthCentroidUp > 0)
            {
                cpUp = ComputeCenterOfPressureApexUp(z0, medianPoint, H);
                float area = ComputeTriangleArea(H, M, P);
                fUp = -RHO * g * area * System.Math.Abs(depthCentroidUp);
            }
        }

        // If lower base is not horizontal, compute apex down part
        if (M.y > L.y)
        {
            float[] waterAtMedian = new float[1];
            _waterProvider.SampleHeightAt(new Vector3[] { medianPoint }, ref waterAtMedian);
            float z0 = waterAtMedian[0] - medianPoint.y;

            //float z0 = _waterProvider.GetHeightAt(medianPoint) - medianPoint.y;
            Vector3 centroidDown = ComputeCentroid(L, M, P);
            float[] waterAtC = new float[1];
            _waterProvider.SampleHeightAt(new Vector3[] { centroidDown }, ref waterAtC);
            float depthCentroidDown = waterAtC[0] - centroidDown.y;
            //float depthCentroidDown = _waterProvider.GetHeightAt(centroidDown) - centroidDown.y;
            if (depthCentroidDown > 0)
            {
                cpDown = ComputeCenterOfPressureApexDown(z0, medianPoint, L);
                float area = ComputeTriangleArea(L, M, P);
                fDown = -RHO * g * area * System.Math.Abs(depthCentroidDown);
            }
        }

        Force force;
        force.force = normal * (fUp + fDown);
        force.appliPoint = ComputeBarycenter(fUp, cpUp, fDown, cpDown);

        return force;
    }

    Vector3 ComputeBarycenter(float alpha, Vector3 A, float beta, Vector3 B)
    {
        if (alpha + beta == 0)
            return new Vector3(0, 0, 0);

        Vector3 AB = B - A;
        return A + beta / (alpha + beta) * AB;
    }

    Vector3 ComputeCenterOfPressureApexUp(float z0, Vector3 median, Vector3 H)
    {
        // z0 and h must be > 0
        // tc ranges from 0 (apex) to 1 (base)

        float h = H.y - median.y;
        float tc = 2 / 3.0f;
        float denom = (6 * z0 + 4 * h);
        if (denom > 0)
            tc = (4 * z0 + 3 * h) / denom;

        Vector3 HToMed = median - H;
        //HToMed.Normalize();
        return H + HToMed * tc;
    }

    Vector3 ComputeCenterOfPressureApexDown(float z0, Vector3 median, Vector3 L)
    {
        // z0 and h must be > 0
        // tc ranges from 0 (base) to 1 (apex)
        float h = median.y - L.y;
        float tc = 1.0f / 3.0f;
        float denom = 6 * z0 + 2 * h;
        if (denom > 0)
            tc = (2 * z0 + h) / denom;

        Vector3 medToL = L - median;
        //HToMed.Normalize();
        return median + medToL * tc;
    }

    Vector3 HorizontalIntersection(Vector3 H, Vector3 M, Vector3 L)
    {
        // Intersection is : 
        // d1 = (H - L) and d2, horizontal line = (x - Mx, 0, z - Mz)  
        // x = Lx + d1x * t and x = Mx + d2x * t
        // y = Ly + d1y * t and y = My 
        // z = Lz + d1z * t and z = Mz + d2z * t
        // => My = Ly + d1y * t
        // => t = (My - Ly) / (Hy - Ly)
        // => deduce x and z

        float t = 0;
        float denom = H.y - L.y;
        if (denom > 0.0f)
            t = (M.y - L.y) / (denom);

        return new Vector3(
            L.x + (H.x - L.x) * t,
            M.y,
            L.z + (H.z - L.z) * t);
    }



    float ComputeTriangleArea(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        Vector3 normal = Vector3.Cross(p2 - p1, p3 - p1);
        float area = 0.5f * normal.magnitude;

        return area;
    }

    bool CompareNormals(Vector3 a, Vector3 b)
    {
        if (Mathf.Sign(a.x) == Mathf.Sign(b.x) && Mathf.Sign(a.y) == Mathf.Sign(b.y) && Mathf.Sign(a.z) == Mathf.Sign(b.z))
            return true;
        return false;
    }
}













//2

//class BoatForcesUnity : MonoBehaviour
//{
//    internal struct VertexAndDepth
//    {
//        public VertexAndDepth(Vector3 vertex, double h)
//        {
//            this.vertex = vertex;
//            this.h = h;
//        }

//        public Vector3 vertex;
//        public double h;
//    }

//    internal class SortedVector3Comparer : IComparer<VertexAndDepth>
//    {
//        public int Compare(VertexAndDepth a, VertexAndDepth b)
//        {
//            int cmp = a.vertex.y.CompareTo(b.vertex.y);
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
//        }
//    }

//    struct DepthTriangle
//    {
//        public DepthTriangle(Vector3 p1, Vector3 p2, Vector3 p3, double depth1, double depth2, double depth3, Vector3 normal)
//        {
//            vertices = new VertexAndDepth[3];
//            vertices[0] = new VertexAndDepth(p1, depth1);
//            vertices[1] = new VertexAndDepth(p2, depth2);
//            vertices[2] = new VertexAndDepth(p3, depth3);

//            _oldIndices = new int[] { 0, 1, 2 };
//            Array.Sort(vertices, _oldIndices, new SortedVector3Comparer());

//            this.normal = normal;
//        }

//        public Vector3 H => vertices[2].vertex;
//        public Vector3 M { get { return vertices[1].vertex; } }
//        public Vector3 L { get { return vertices[0].vertex; } }

//        public double depthH => vertices[2].h;
//        public double depthM => vertices[1].h;
//        public double depthL => vertices[0].h;

//        public Vector3 p0 => vertices[_oldIndices[0]].vertex;
//        public Vector3 p1 => vertices[_oldIndices[1]].vertex;
//        public Vector3 p2 => vertices[_oldIndices[2]].vertex;

//        public Vector3 normal;

//        private VertexAndDepth[] vertices;

//        private int[] _oldIndices;
//        //private double[] depths;
//        public void Affect(Vector3 point)
//        {

//        }
//    }

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

//    struct TriangleAndVertices
//    {
//        public int triIndex;
//        public VertexAndDepth hl;
//        public VertexAndDepth hm;
//        public VertexAndDepth hh;
//    }

//    private DepthTriangle[] _trianglesAndVertices;
//    private Mesh _mesh;
//    List<DepthTriangle> _submergedTriangles;
//    List<DepthTriangle> _emergedTriangles;

//    public GameObject _underwaterGameObject;
//    Mesh _underwaterMesh;

//    float _surface = 0;
//    float _submergedSurface = 0;
//    //public float _mass = 10;

//    IWaterProvider _waterProvider;

//    Body _body;
//    BodyState _bodyState;

//    public float _mass = 100;
//    public Vector3 _cogOffset = new Vector3(0, 0, 0);
//    public Vector3 _diagInertia = new Vector3(50, 50, 50);

//    public RefFrame _refFrame = RefFrame.WORLD;

//    protected Rigidbody _rigidbody;

//    public bool _computeHydrodynamics = true;

//    // Start is called before the first frame update
//    void Start()
//    {
//        _mesh = GetComponent<MeshFilter>().sharedMesh;
//        _trianglesAndVertices = new DepthTriangle[_mesh.triangles.Length / 3];
//        _submergedTriangles = new List<DepthTriangle>();


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

//            //Vector3 normal = Vector3.Cross(worldVertex1 - worldVertex0, worldVertex2 - worldVertex0);
//            _surface += ComputeTriangleArea(worldVertex0, worldVertex1, worldVertex2);// 0.5f * normal.magnitude;
//        }

//        //_waterProvider = new SimpleWaterProvider(0);
//        //_waterProvider = new SinWaterProvider();
//        _waterProvider = new CrestWaterProvider(1);

//        //
//        _rigidbody = GetComponent<Rigidbody>();
//        _rigidbody.useGravity = false;
//        _rigidbody.detectCollisions = false;
//        _rigidbody.inertiaTensor = _diagInertia;
//        _rigidbody.mass = _mass;
//        //_rigidbody.centerOfMass = _cogOffset;
//    }

//    // Update is called once per frame
//    void Update()
//    {
//    }

//    void FixedUpdate()
//    {
//        _waterProvider.SetSamplingTime(Time.fixedTime);

//        List<Force> allForces = new List<Force>();
//        Vector3 sumForces = new Vector3();
//        //(Force forces, Vector3 torques) = ComputeWeight();
//        Force fweight = ComputeWeight();
//        allForces.Add(fweight);

//        List<Force> forcesB = ComputeBuoyancy();
//        allForces.AddRange(forcesB);

//        //Force forceTheoriB = ComputeTheoriticalBuoyancy();
//        //allForces.Add(forceTheoriB);



//        //allForces.Add(damping);

//        if(_computeHydrodynamics)
//        {
//            allForces.AddRange(ComputeEmpiricalDrag());
//            //allForces.AddRange(ComputeHydrodynamics());
//        }

//        foreach (Force fb in allForces)
//        {
//            if (_refFrame == RefFrame.WORLD)
//                _rigidbody.AddForceAtPosition(fb.force, fb.appliPoint);
//            else
//                _rigidbody.AddForceAtPosition(transform.InverseTransformDirection(fb.force), transform.InverseTransformPoint(fb.appliPoint));
//            sumForces += fb.force;
//        }

//        Debug.Log("unity " + sumForces + " and " + _rigidbody.velocity + " and " + transform.position);

//        (Vector3 damping2, Vector3 torque) = ComputeDamping2(_submergedSurface, _surface, _rigidbody.velocity, _rigidbody.angularVelocity, transform.position);
//        //(Vector3 damping2, Vector3 torque) = ComputeDamping3(_rigidbody.velocity, _rigidbody.angularVelocity);
//        //_rigidbody.AddForce(damping2);
//        //_rigidbody.AddTorque(torque);

//        //Vector3 speed;
//        //if (_refFrame == RefFrame.WORLD)
//        //    speed = _bodyState.speed;
//        //else
//        //    speed = _bodyState.speed_body;
//        //Force damping = ComputeDamping(_submergedSurface, _surface, speed);
//        //force.Add(forces);

//        //force.Add(forceTheoriB);
//        //force.Add(damping);
//        //Debug.Log(forcesB.force);
//        //forces.force += forcesB.force + damping.force;
//    }

//    /*    (Vector3 force, Vector3 torque) ComputeWeight()
//        {
//            Force force = new Force();
//            Vector3 torque = new Vector3();

//            Vector3 G = new Vector3(0, -9.81f, 0);
//            force.force = _mass * G;

//            //Debug.Log(force.y);

//            return (force: force, torque: torque);
//        }*/

//    Force ComputeWeight()
//    {
//        Force force = new Force();
//        Vector3 torque = new Vector3();

//        force.force = _mass * UnityPhysicsConstants.G;
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

//        //Ok ?
//        force.appliPoint = cog;

//        return force;
//    }

//    (Vector3, Vector3)  ComputeDampingBis(float submergedSurface, float totalSurface, Vector3 speed_value, Vector3 angularSpeed)
//    {
//        const float Cdamp = 1000;
//        float rs = submergedSurface / totalSurface;
//        Vector3 force = -Cdamp * rs * speed_value;

//        //Ok ?
//        Vector3 torque = -Cdamp * 0.0001f * rs * angularSpeed;

//        return (force, torque);
//    }

//    public (Vector3, Vector3) ComputeDamping2(float submergedSurface, float totalSurface, Vector3 speed_value, Vector3 angularSpeed, Vector3 cog)
//    {
//        // Linear drag coefficients
//        float cDampL1 = 1000f;
//        float cDampR1 = 100f;

//        // Quadratic drag coefficients
//        float cDampL2 = 100f;
//        float cDampR2 = 10f;

//        float area = totalSurface;
//        float subArea = submergedSurface;
//        float rs = subArea / area;

//        // Force
//        float linSpeed = speed_value.magnitude;
//        float cL = -rs * (cDampL1 + cDampL2 * linSpeed);
//        Vector3 force = speed_value * cL;

//        float angSpeed = angularSpeed.magnitude;
//        float cR = -rs * (cDampR1 + cDampR2 * angSpeed);
//        Vector3 torque = angularSpeed * cR;
//        return (force, torque);
//    }

//    public (Vector3, Vector3) ComputeDamping3(Vector3 speed_body, Vector3 angularSpeed)
//    {
//        Vector3 force;
//        float RHO = UnityPhysicsConstants.RHO;
//        float referenceArea = _submergedSurface / 4;
//        float Cdx = 1.05f*10;
//        float Cdy = 1.5f*10;
//        float Cdz = 0.5f*10;
//        force.x = -0.5f * RHO * referenceArea * Cdx * Mathf.Abs(speed_body.x) * speed_body.x;
//        force.y = -0.5f * RHO * referenceArea * Cdy * Mathf.Abs(speed_body.y) * speed_body.y;
//        force.z = -0.5f * RHO * referenceArea * Cdz * Mathf.Abs(speed_body.z) * speed_body.z;

//        Vector3 torque;
//        torque.x = -0.5f * RHO * referenceArea * Cdx/5 * Mathf.Abs(angularSpeed.x) * angularSpeed.x;
//        torque.y = -0.5f * RHO * referenceArea * Cdy/5 * Mathf.Abs(angularSpeed.y) * angularSpeed.y;
//        torque.z = -0.5f * RHO * referenceArea * Cdz/5 * Mathf.Abs(angularSpeed.z) * angularSpeed.z;

//        return (force, torque);
//    }

//    Force ComputeViscousDrag(Vector3 speedAtTriangle, float area, Vector3 normal, Vector3 center)
//    {
//        Force force;

//        float viscousDragCoeff = 1.0f;
//        float cosTheta = Vector3.Dot(speedAtTriangle.normalized, normal);
//        if (cosTheta >= 0)
//            force.force = -(1 - cosTheta) * viscousDragCoeff * speedAtTriangle * area;
//        else
//            force.force = (1 - cosTheta) * viscousDragCoeff * speedAtTriangle * area;
//        force.appliPoint = center;
//        return force;

//        float RHO = UnityPhysicsConstants.RHO;

//        double viscosity = 1.0533E-6f;
//        double L = 2.5 * 2 + 1 * 2; //TODO
//        double speedMagnitude = speedAtTriangle.magnitude;
//        double Rn = speedMagnitude * L / viscosity;
//        double Cf = 0.075 / Math.Pow(Math.Log10(Rn) - 2, 2);
//        //Vector3 vi = speedAtTriangle - normal * Vector3.Dot(speedAtTriangle.normalized, normal);
//        Vector3 vti = speedAtTriangle - normal * Vector3.Dot(speedAtTriangle.normalized, normal);
//        Vector3 ufi = -vti.normalized;
//        Vector3 vfi = (float)speedMagnitude * ufi;
//        Vector3 drag = 0.5f * RHO * (float)Cf * area * vfi.magnitude * vfi;

//        force.force = drag;
//        force.appliPoint = center;

//        return force;
//    }

//    Force ComputePressureDrag(Vector3 speedAtTriangle, float area, Vector3 normal, Vector3 center)
//    {
//        Force force;
//        Vector3 Fd;
//        float Cpd1 = 1000;
//        float Cpd2 = 1000;
//        float vi = speedAtTriangle.magnitude;
//        float vr = 1;
//        float fp = 0.4f;
//        float cosTheta = Vector3.Dot(speedAtTriangle.normalized, normal);
//        float Csd1 = 1000;
//        float Csd2 = 1000;
//        float fs = 0.4f;
//        if (cosTheta >= 0)
//        {
//            Fd = -(Cpd1 * vi / vr + Cpd2 * (float)Math.Pow(vi / vr, 2)) * area * (float)Math.Pow(cosTheta, fp) * normal;
//        }
//        else
//        {
//            Fd = (Csd1 * vi / vr + Csd2 * (float)Math.Pow(vi / vr, 2)) * area * (float)Math.Pow(-cosTheta, fs) * normal;
//        }

//        force.force = Fd;
//        force.appliPoint = center;

//        return force;
//    }

//    List<Force> ComputeEmpiricalDrag()
//    {
//        List<Force> forces = new List<Force>(); 

//        foreach (DepthTriangle triangle in _submergedTriangles)
//        {
//            float RHO = UnityPhysicsConstants.RHO;
//            float area = ComputeTriangleArea(triangle.H, triangle.M, triangle.L);
//            Vector3 center = ComputeCentroid(triangle.H, triangle.M, triangle.L);

//            float depthCenter = _waterProvider.GetHeightAt(center) - center.y;
//            if (depthCenter < 0)
//                continue;

//            //Test drag
//            Vector3 p = center - _rigidbody.worldCenterOfMass;
//            float VelocityDotPower = 1;
//            float DynamicForcePower = 1;
//            Vector3 dynamicForce;
//            Vector3 velocity = _rigidbody.velocity + Vector3.Cross(_rigidbody.angularVelocity, p);
//            float velocityMagnitude = velocity.magnitude;
//            float DynamicForceFactor = 1;
//            float dot = Vector3.Dot(triangle.normal, velocity.normalized);
//            float densityArea = RHO * area;
//            float SkinDrag = 1;
//            {
//                // Dynamic force
//                if (dot < -0.0001f || dot > 0.0001f)
//                {
//                    dot = Mathf.Sign(dot) * Mathf.Pow(Mathf.Abs(dot), VelocityDotPower);
//                }

//                if (DynamicForcePower < 0.9999f || DynamicForcePower > 1.0001f)
//                {
//                    dynamicForce = -dot * Mathf.Pow(velocityMagnitude, DynamicForcePower * 1.0f) //ObjDynamicForcePowerCoeffs[objIndex])
//                                        * densityArea * DynamicForceFactor * 1.0f * triangle.normal; //ObjDynamicForceCoeffs[objIndex] * normal;
//                }
//                else
//                {
//                    dynamicForce = -dot * velocityMagnitude * densityArea * DynamicForceFactor * 1.0f * triangle.normal;// ObjDynamicForceCoeffs[objIndex] * normal;
//                }
//            }

//            {
//                float absDot = dot < 0 ? -dot : dot;
//                dynamicForce += (1f - absDot) * SkinDrag * 1.0f * densityArea * -velocity;// ObjSkinFrictionDragCoeffs[objIndex] * densityArea * -velocity;
//            }

//            Force drag;
//            drag.force = dynamicForce;
//            drag.appliPoint = center;
//            forces.Add(drag);
//        }

//        return forces;
//    }

//    List<Force> ComputeHydrodynamics()
//    {
//        List<Force> forces = new List<Force>();

//        foreach (DepthTriangle triangle in _submergedTriangles)
//        {
//            float area = ComputeTriangleArea(triangle.H, triangle.M, triangle.L);
//            Vector3 center = ComputeCentroid(triangle.H, triangle.M, triangle.L);
//            Vector3 p = center - _rigidbody.worldCenterOfMass;
//            Vector3 speedAtTriangle = _rigidbody.velocity + Vector3.Cross(_rigidbody.angularVelocity, p);
//            forces.Add(ComputeViscousDrag(speedAtTriangle, area, triangle.normal, center));
//            forces.Add(ComputePressureDrag(speedAtTriangle, area, triangle.normal, center));
//        }



//        //Vector3 speed;
//        //if (_refFrame == RefFrame.WORLD)
//        //    speed = _rigidbody.velocity;
//        //else
//        //    speed = transform.InverseTransformDirection(_rigidbody.velocity);
//        //Force damping = ComputeDamping(_submergedSurface, _surface, speed, transform.position);
//        //(Vector3 dbis, Vector3 dtbis) = ComputeDampingBis(_submergedSurface, _surface, speed, _rigidbody.angularVelocity);
//        ////_rigidbody.AddForce(dbis);
//        ////_rigidbody.AddTorque(dtbis);

//        return forces;
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
//            Vector3 G = -UnityPhysicsConstants.G;
//            float RHO = UnityPhysicsConstants.RHO;
//            force.force = RHO * volume * G;
//            if (lowY < -1)
//                force.appliPoint = this.transform.position;
//            else
//                force.appliPoint = new Vector3(transform.position.x, lowY / 2.0f, transform.position.z);
//        }

//        return force;
//    }



//    List<Force> ComputeBuoyancy()
//    {
//        Vector3[] vertices = _mesh.vertices;

//        _submergedTriangles.Clear();

//        //Vector3[] samplingList = new Vector3[_trianglesAndVertices.Length * 3];
//        //for (int i = 0; i < _trianglesAndVertices.Length; i++)
//        //{
//        //    int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
//        //    Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
//        //    Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
//        //    Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

//        //    samplingList[i * 3] = worldVertex0;
//        //    samplingList[i * 3 + 1] = worldVertex1;
//        //    samplingList[i * 3 + 2] = worldVertex2;
//        //}

//        //_waterProvider.UpdateSamplingList(samplingList);

//        for (int i = 0; i < _trianglesAndVertices.Length; i++)
//        {
//            int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
//            Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
//            Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
//            Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

//            //See if we keep
//            //Vector3[] samplingList = new Vector3[_trianglesAndVertices.Length * 3];
//            Vector3[] samplingList = new Vector3[3];
//            samplingList[0] = worldVertex0;
//            samplingList[1] = worldVertex1;
//            samplingList[2] = worldVertex2;
//            _waterProvider.UpdateSamplingList(samplingList);
//            //

//            float depth0 = worldVertex0.y - _waterProvider.GetHeightAt(0); //i * 3);
//            float depth1 = worldVertex1.y - _waterProvider.GetHeightAt(1);// i * 3 + 1);
//            float depth2 = worldVertex2.y - _waterProvider.GetHeightAt(2);//< i * 3 + 2);

//            Vector3 currentTriNormal = (_mesh.normals[indices[0]] + _mesh.normals[indices[1]] + _mesh.normals[indices[2]]) / 3.0f;
//            //should take care of scaling factor...
//            currentTriNormal = transform.TransformDirection(currentTriNormal);
//            currentTriNormal.Normalize();

//            DepthTriangle triangle = new DepthTriangle(worldVertex0, worldVertex1, worldVertex2,
//                                                       depth0, depth1, depth2,
//                                                       currentTriNormal);
//            _trianglesAndVertices[i] = triangle;

//            // Cutting triangles
//            if (triangle.depthH > 0 && triangle.depthL < 0)
//            {
//                //Vector3 ml = _trianglesAndVertices[i].hl.vertex - _trianglesAndVertices[i].hm.vertex;

//                // 2 underwater
//                if (triangle.depthL < 0 && triangle.depthM < 0)
//                {
//                    Vector3 lh = triangle.H - triangle.L;
//                    Vector3 mh = triangle.H - triangle.M;

//                    float tm = (float)(-triangle.depthM / (triangle.depthH - triangle.depthM));
//                    Vector3 vecIm = tm * mh;
//                    Vector3 im = vecIm + triangle.M;

//                    float tl = (float)(-triangle.depthL / (triangle.depthH - triangle.depthL));
//                    Vector3 vecIl = tl * lh;
//                    Vector3 il = vecIl + triangle.L;

//                    Vector3 tmpNormal = Vector3.Cross(triangle.M - triangle.L, triangle.L - il);

//                    _waterProvider.UpdateSamplingList(new Vector3[] { il, im });
//                    float depthIl = il.y - _waterProvider.GetHeightAt(0);
//                    float depthIm = im.y - _waterProvider.GetHeightAt(1);
//                    if (CompareNormals(tmpNormal, currentTriNormal))
//                    {
//                        _submergedTriangles.Add(new DepthTriangle(il, triangle.M, triangle.L, depthIl, triangle.depthM, triangle.depthL, currentTriNormal));
//                        _submergedTriangles.Add(new DepthTriangle(im, triangle.M, il, depthIm, triangle.depthM, depthIl, currentTriNormal));
//                    }
//                    else
//                    {
//                        _submergedTriangles.Add(new DepthTriangle(il, triangle.L, triangle.M, depthIl, triangle.depthL, triangle.depthM, currentTriNormal));
//                        _submergedTriangles.Add(new DepthTriangle(im, il, triangle.M, depthIm, depthIl, triangle.depthM, currentTriNormal));
//                    }

//                } //1 underwater
//                else if (triangle.depthM > 0 && triangle.depthL < 0)
//                {
//                    Vector3 lm = triangle.M - triangle.L;
//                    Vector3 lh = triangle.H - triangle.L;

//                    float tm = (float)(-triangle.depthL / (triangle.depthM - triangle.depthL));
//                    Vector3 vecJm = tm * lm;
//                    Vector3 jm = vecJm + triangle.L;

//                    float th = (float)(-triangle.depthL / (triangle.depthH - triangle.depthL));
//                    Vector3 vecJh = th * lh;
//                    Vector3 jh = vecJh + triangle.L;

//                    Vector3 tmpNormal = Vector3.Cross(jm - jh, triangle.L - jh);

//                    _waterProvider.UpdateSamplingList(new Vector3[] { jm, jh });
//                    float depthJm = jm.y - _waterProvider.GetHeightAt(0);
//                    float depthJh = jh.y - _waterProvider.GetHeightAt(1);


//                    if (!CompareNormals(tmpNormal, currentTriNormal))
//                        _submergedTriangles.Add(new DepthTriangle(jh, triangle.L, jm, depthJh, triangle.depthL, depthJm, currentTriNormal));
//                    else
//                        _submergedTriangles.Add(new DepthTriangle(jh, jm, triangle.L, depthJh, depthJm, triangle.depthL, currentTriNormal));
//                }
//            }
//            else if (triangle.depthH < 0 && triangle.depthM < 0 && triangle.depthL < 0)
//                _submergedTriangles.Add(new DepthTriangle(worldVertex0, worldVertex1, worldVertex2, depth0, depth1, depth2, currentTriNormal));
//        }

//        List<Vector3> uvertices = new List<Vector3>();
//        List<int> utri = new List<int>();
//        List<Color> ucolor = new List<Color>();
//        List<Vector3> unormals = new List<Vector3>();
//        Color col = Color.yellow;

//        List<Force> forces = new List<Force>();

//        _submergedSurface = 0;
//        foreach (DepthTriangle triangle in _submergedTriangles)
//        {
//            float RHO = UnityPhysicsConstants.RHO;
//            float absG = -UnityPhysicsConstants.G.y;
//            float area = ComputeTriangleArea(triangle.H, triangle.M, triangle.L);
//            //Vector3 normal = Vector3.Cross(triangle.H - triangle.L, triangle.M - triangle.L);
//            //float area = 0.5f * normal.magnitude;
//            _submergedSurface += area;
//            Vector3 center = ComputeCentroid(triangle.H, triangle.M, triangle.L);
//            //Debug.Log(center.y);
//            float depthCenter = _waterProvider.GetHeightAt(center) - center.y;
//            if (depthCenter < 0)
//                continue;

//            ///Force force = ComputeBuoyancyAtCentroid(triangle.H, triangle.M, triangle.L, triangle.normal, center, depthCenter);      
//            //forces.Add(force);


//            //Force force1 = ComputeCenterOfPressure2(triangle.H, triangle.M, triangle.L, triangle.normal);
//            //force1.force.x = force.force.z = 0;
//            //forces.Add(force1);

//            Force force = ComputeBuoyancyAtCenterOfPressure(triangle.H, triangle.M, triangle.L, triangle.normal);
//            //force.force.x = force.force.z = 0;
//            forces.Add(force);

//            Debug.DrawLine(force.appliPoint, force.appliPoint + triangle.normal.normalized * 3, Color.magenta);




//            //force.force += transform.InverseTransformDirection(tmpForce);
//            //force.appliPoint = transform.InverseTransformPoint(center);

//            uvertices.Add(transform.InverseTransformPoint(triangle.p0));
//            unormals.Add(transform.InverseTransformDirection(triangle.normal));
//            utri.Add(uvertices.Count - 1);
//            ucolor.Add(col);
//            uvertices.Add(transform.InverseTransformPoint(triangle.p1));
//            unormals.Add(transform.InverseTransformDirection(triangle.normal));
//            utri.Add(uvertices.Count - 1);
//            ucolor.Add(col);
//            uvertices.Add(transform.InverseTransformPoint(triangle.p2));
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

//    Force ComputeBuoyancyAtCentroid(Vector3 H, Vector3 M, Vector3 L, Vector3 normal, Vector3 centroid, float depthCentroid)
//    {
//        Force force;
//        float RHO = UnityPhysicsConstants.RHO;
//        float absG = Mathf.Abs(UnityPhysicsConstants.G.y);

//        float area = ComputeTriangleArea(H, M, L);

//        Vector3 tmpForce = -RHO * absG * area * normal * System.Math.Abs(depthCentroid);
//        //Debug.Log(center.y);

//        force.force = tmpForce;
//        force.appliPoint = centroid;

//        return force;
//    }

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
//    Force ComputeCenterOfPressure2(Vector3 H, Vector3 M, Vector3 L, Vector3 normal)
//    {
//        float fluidDensity = UnityPhysicsConstants.RHO; // kg m^-3 
//        float gravity = UnityPhysicsConstants.G.y;     // m s^-1

//        Vector3 C = ComputeCentroid(H, M, L);

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
//);
//    }

//    Vector3 MidPoint(Vector3 a, Vector3 b)
//    {
//        return (a + b) / 2.0f;
//    }



//    Force ComputeBuoyancyAtCenterOfPressure(Vector3 H, Vector3 M, Vector3 L, Vector3 normal)
//    {
//        float RHO = UnityPhysicsConstants.RHO;
//        float g = -UnityPhysicsConstants.G.y;

//        Vector3 P = HorizontalIntersection(H, M, L);
//        Vector3 medianPoint = MidPoint(P, M);

//        float fUp = 0;
//        float fDown = 0;
//        Vector3 cpUp = new Vector3(0, 0, 0);
//        Vector3 cpDown = new Vector3(0, 0, 0);

//        //
//        //Vector3 centroid = ComputeCentroid(H, M, L);
//        //float depthC = _waterProvider.GetHeightAt(centroid) - centroid.y;

//        // If upper base is not horizontal, compute apex up part
//        // Use >= to handle horizontal triangles
//        if (H.y >= M.y)
//        {
//            float z0 = _waterProvider.GetHeightAt(H) - H.y;
//            Vector3 centroidUp = ComputeCentroid(H, M, P);
//            float depthCentroidUp = _waterProvider.GetHeightAt(centroidUp) - centroidUp.y;
//            if (depthCentroidUp > 0)
//            {
//                cpUp = ComputeCenterOfPressureApexUp(z0, medianPoint, H);
//                float area = ComputeTriangleArea(H, M, P);
//                fUp = -RHO * g * area * System.Math.Abs(depthCentroidUp);
//            }
//        }

//        // If lower base is not horizontal, compute apex down part
//        if (M.y > L.y)
//        {
//            float z0 = _waterProvider.GetHeightAt(medianPoint) - medianPoint.y;
//            Vector3 centroidDown = ComputeCentroid(L, M, P);
//            float depthCentroidDown= _waterProvider.GetHeightAt(centroidDown) - centroidDown.y;
//            if (depthCentroidDown > 0)
//            {
//                cpDown = ComputeCenterOfPressureApexDown(z0, medianPoint, L);
//                float area = ComputeTriangleArea(L, M, P);
//                fDown = -RHO * g * area * System.Math.Abs(depthCentroidDown);
//            }
//        }

//        Force force;
//        force.force = normal * (fUp + fDown);
//        force.appliPoint = ComputeBarycenter(fUp, cpUp, fDown, cpDown);

//        return force;
//    }

//    Vector3 ComputeBarycenter(float alpha, Vector3 A, float beta, Vector3 B)
//    {
//        if (alpha + beta == 0)
//            return new Vector3(0, 0, 0);

//        Vector3 AB = B - A;
//        return A + beta / (alpha + beta) * AB;
//    }

//    Vector3 ComputeCenterOfPressureApexUp(float z0, Vector3 median, Vector3 H)
//    {
//        // z0 and h must be > 0
//        // tc ranges from 0 (apex) to 1 (base)

//        float h = H.y - median.y;
//        float tc = 2 / 3.0f; 
//        float denom = (6 * z0 + 4 * h);
//        if (denom > 0)
//            tc = (4 * z0 + 3 * h) / denom; 

//        Vector3 HToMed = median - H;
//        //HToMed.Normalize();
//        return H + HToMed * tc;
//    }

//    Vector3 ComputeCenterOfPressureApexDown(float z0, Vector3 median, Vector3 L)
//    {
//        // z0 and h must be > 0
//        // tc ranges from 0 (base) to 1 (apex)
//        float h = median.y - L.y;
//        float tc = 1.0f / 3.0f;
//        float denom = 6 * z0 + 2 * h;
//        if(denom > 0)
//            tc = (2 * z0 + h) / denom;

//        Vector3 medToL = L - median;
//        //HToMed.Normalize();
//        return median + medToL * tc;
//    }

//    Vector3 HorizontalIntersection(Vector3 H, Vector3 M, Vector3 L)
//    {
//        // Intersection is : 
//        // d1 = (H - L) and d2, horizontal line = (x - Mx, 0, z - Mz)  
//        // x = Lx + d1x * t and x = Mx + d2x * t
//        // y = Ly + d1y * t and y = My 
//        // z = Lz + d1z * t and z = Mz + d2z * t
//        // => My = Ly + d1y * t
//        // => t = (My - Ly) / (Hy - Ly)
//        // => deduce x and z

//        float t = 0;
//        float denom = H.y - L.y;
//        if (denom > 0.0f)
//            t = (M.y - L.y) / (denom);

//        return new Vector3(
//            L.x + (H.x - L.x) * t,
//            M.y,
//            L.z + (H.z - L.z) * t);
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
//}


// 3

//class BoatForcesUnity : MonoBehaviour
//{
//    struct Triangle2
//    {
//        public Vector3 H;
//        public Vector3 M;
//        public Vector3 L;

//        public double[] depth;
//    }

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
//    public Vector3 _cogOffset = new Vector3(0, 0, 0);
//    public Vector3 _diagInertia = new Vector3(50, 50, 50);

//    public RefFrame _refFrame = RefFrame.WORLD;

//    protected Rigidbody _rigidbody;

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
//        //_waterProvider = new CrestWaterProvider(1);
//        //GetComponent<ChronoPhysicsManager.in

//        //
//        _rigidbody = GetComponent<Rigidbody>();
//        _rigidbody.useGravity = false;
//        _rigidbody.detectCollisions = false;
//        _rigidbody.inertiaTensor = _diagInertia;
//        _rigidbody.mass = _mass;
//        //_rigidbody.centerOfMass = _cogOffset;
//    }

//    // Update is called once per frame
//    void Update()
//    {
//    }

//    void FixedUpdate()
//    {
//        List<Force> allForces = new List<Force>();
//        Vector3 sumForces = new Vector3();
//        //(Force forces, Vector3 torques) = ComputeWeight();
//        Force fweight = ComputeWeight();
//        allForces.Add(fweight);

//        List<Force> forcesB = ComputeBuoyancy();
//        allForces.AddRange(forcesB);

//        //Force forceTheoriB = ComputeTheoriticalBuoyancy();
//        //allForces.Add(forceTheoriB);

//        Vector3 speed;
//        if (_refFrame == RefFrame.WORLD)
//            speed = _rigidbody.velocity;
//        else
//            speed = transform.InverseTransformDirection(_rigidbody.velocity);
//        Force damping = ComputeDamping(_submergedSurface, _surface, speed);

//        //allForces.Add(damping);

//        foreach (Force fb in allForces)
//        {
//            if (_refFrame == RefFrame.WORLD)
//                _rigidbody.AddForceAtPosition(fb.force, fb.appliPoint);
//            else
//                _rigidbody.AddForceAtPosition(transform.TransformDirection(fb.force), transform.TransformPoint(fb.appliPoint));
//            sumForces += fb.force;
//        }

//        Debug.Log("unity " + sumForces + " and " + _rigidbody.velocity + " and " + transform.position);



//        //Vector3 speed;
//        //if (_refFrame == RefFrame.WORLD)
//        //    speed = _bodyState.speed;
//        //else
//        //    speed = _bodyState.speed_body;
//        //Force damping = ComputeDamping(_submergedSurface, _surface, speed);
//        //force.Add(forces);

//        //force.Add(forceTheoriB);
//        //force.Add(damping);
//        //Debug.Log(forcesB.force);
//        //forces.force += forcesB.force + damping.force;
//        //ChronoPhysicsManager manager = GetComponent<ChronoPhysicsManager>();
//        //manager.SetMass(_mass);
//        ////(Force forces, Vector3 torques) = ComputeWeight();
//        //Force forces = ComputeWeight();
//        //(Force forcesB, Vector3 torquesB) = ComputeBuoyancy();
//        //Force damping = ComputeDamping(_submergedSurface, _surface, manager.GetSpeedBody());

//        //forces.force += forcesB.force + damping.force;

//        //manager.UpdateForces(forces, Time.deltaTime);
//    }

//    /*    (Vector3 force, Vector3 torque) ComputeWeight()
//        {
//            Force force = new Force();
//            Vector3 torque = new Vector3();

//            Vector3 G = new Vector3(0, -9.81f, 0);
//            force.force = _mass * G;

//            //Debug.Log(force.y);

//            return (force: force, torque: torque);
//        }*/

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
//            if(lowY < -1)
//                force.appliPoint = this.transform.position;
//            else
//                force.appliPoint = new Vector3(transform.position.x, lowY / 2.0f, transform.position.z);
//        }

//        return force;
//    }

//    List<Force> ComputeBuoyancy()
//    {
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
//            SortedSet<VertexAndDepth> h = new SortedSet<VertexAndDepth>(new SortedVector3Comparer());
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
//            foreach (VertexAndDepth val in h)
//            {
//                if (cpt == 0)
//                    _trianglesAndVertices[i].hl = val;
//                if (cpt == 1)
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
//                else if (_trianglesAndVertices[i].hm.h > 0 && _trianglesAndVertices[i].hl.h < 0)
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
//            else if (_trianglesAndVertices[i].hh.h < 0 && _trianglesAndVertices[i].hm.h < 0 && _trianglesAndVertices[i].hl.h < 0)
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

//            //force1.force.x = force.force.z = 0;
//            forces.Add(force1);

//            Debug.DrawLine(force1.appliPoint, force1.appliPoint + triangle.normal.normalized * 3, Color.blue);
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

//    internal class SortedVector3Comparer2 : IComparer<Vector3>
//    {
//        public int Compare(Vector3 a, Vector3 b)
//        {

//            if(a.y == b.y)
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
//);
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
//}

