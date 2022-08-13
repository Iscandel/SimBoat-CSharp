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


class BoatForcesUnity : MonoBehaviour
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
            vert = (VertexAndDepth[])vertices.Clone();
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

    private DepthTriangle[] _trianglesAndVertices;
    private Mesh _mesh;
    List<DepthTriangle> _submergedTriangles;
    List<DepthTriangle> _emergedTriangles;

    public GameObject _underwaterGameObject;
    Mesh _underwaterMesh;

    float _surface = 0;
    float _submergedSurface = 0;

    IWaterProvider _waterProvider;

    Body _body;
    BodyState _bodyState;

    public float _mass = 100;
    public Vector3 _cogOffset = new Vector3(0, 0, 0);
    public Vector3 _diagInertia = new Vector3(50, 50, 50);

    public RefFrame _refFrame = RefFrame.WORLD;

    protected Rigidbody _rigidbody;

    public bool _computeHydrodynamics = true;

    public bool _isDebug = true;

    public bool _approximateMode = true;

    //
    //public bool _fixTimeScale = false;

    private float _rudderAngle;
    private float _thrust;
    public Vector3 _thrustAppliPoint; // local coords

    public float _maxThrust = 12000;
    public float _maxRudderAngle = 15;

    // Start is called before the first frame update
    void Start()
    {
        _mesh = GetComponent<MeshFilter>().sharedMesh;
        _trianglesAndVertices = new DepthTriangle[_mesh.triangles.Length / 3];
        _submergedTriangles = new List<DepthTriangle>();

        _underwaterMesh = this.transform.GetChild(0).GetComponent<MeshFilter>().mesh;//.AddComponent<GameObject>(); 

        Vector3[] vertices = _mesh.vertices;
        for (int i = 0; i < _trianglesAndVertices.Length; i++)
        {
            int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
            Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
            Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
            Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

            _surface += GeometryTools.ComputeTriangleArea(worldVertex0, worldVertex1, worldVertex2);
        }

        //_waterProvider = new SimpleWaterProvider(0);
        //_waterProvider = new SinWaterProvider();
        _waterProvider = new CrestWaterProvider(0);
        ((CrestWaterProvider)_waterProvider).SetNumberOfQueries(1);

        //
        _rigidbody = GetComponent<Rigidbody>();
        _rigidbody.useGravity = false;
        _rigidbody.detectCollisions = false;
        _rigidbody.inertiaTensor = _diagInertia;
        _rigidbody.mass = _mass;
        //_rigidbody.centerOfMass = _cogOffset;

        _rudderAngle = 0;
        _thrust = 0;
        _thrustAppliPoint = new Vector3(0, -0.4f, -1.9f);
}

    // Update is called once per frame
    void Update()
    {
        float thrust = 0;
        const float maxThrust = 12000;
        //Debug.Log("thrust " + thrust);

        if (Input.GetKey(KeyCode.Z))
            thrust = 1;
        else thrust = Input.GetAxis("Accelerate");

        if(transform.position.y < 1.8)
            _thrust = thrust * maxThrust;
        else
            _thrust = 0;

        //
        float angle = 0;
        if (Input.GetKey(KeyCode.D))
            angle = -1;
        else if (Input.GetKey(KeyCode.Q))
            angle = 1;
        else
            angle = -Input.GetAxis("Horizontal");

        Debug.Log("Horizontal " + angle);
        _rudderAngle = angle * _maxRudderAngle;

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

        bool status = true;
        List<Force> forcesB = ComputeBuoyancy(ref status);
        if (!status)
            return;
        allForces.AddRange(forcesB);

        //Force forceTheoriB = ComputeTheoriticalBuoyancy();
        //allForces.Add(forceTheoriB);

        Force thrust = ComputeThrustForce();
        allForces.Add(thrust);
        //Debug.Log("THRUST " + thrust.force);

        //allForces.Add(damping);

        if (_computeHydrodynamics)
        {
            allForces.AddRange(ComputeEmpiricalDrag());
            //allForces.AddRange(ComputeHydrodynamics());        
        }

        if (allForces[allForces.Count - 1].force.y > 100000)
            Debug.Log(allForces[allForces.Count - 1].force.y);

        foreach (Force fb in allForces)
        {
            if (_refFrame == RefFrame.WORLD)
                _rigidbody.AddForceAtPosition(fb.force, fb.appliPoint);
            else
                _rigidbody.AddForceAtPosition(transform.InverseTransformDirection(fb.force), transform.InverseTransformPoint(fb.appliPoint));
            sumForces += fb.force;
        }

        ClearDebugMesh();
        if (_isDebug)
        {
            ShowDebugMesh();
        }

        //Debug.Log("unity " + sumForces + " and " + _rigidbody.velocity + " and " + transform.position);

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
    
    Force ComputeThrustForce()
    {
        //Vector3 thrustDirection = new Vector3(0, 0.10f, 0.90f);
        Vector3 thrustDirection = new Vector3(0, 0, 1);
        thrustDirection.Normalize();

        Quaternion rotRudder = Quaternion.Euler(0, _rudderAngle, 0);
        thrustDirection = rotRudder * thrustDirection;

        Vector3 thrustForce = transform.rotation * thrustDirection *_thrust;

        Force force;
        force.force = thrustForce;
        force.appliPoint = transform.TransformPoint(_thrustAppliPoint);

        return force;
    }

    void ClearDebugMesh()
    {
        _underwaterMesh.Clear();
    }

    void ShowDebugMesh()
    {
        List<Vector3> uvertices = new List<Vector3>();
        List<int> utri = new List<int>();
        List<Color> ucolor = new List<Color>();
        List<Vector3> unormals = new List<Vector3>();
        Color col = Color.yellow;

        List<Force> forces = new List<Force>();

        _submergedSurface = 0;
        foreach (DepthTriangle triangle in _submergedTriangles)
        {
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


            _underwaterMesh.Clear();
            _underwaterMesh.name = "underwater mesh";
            _underwaterMesh.vertices = uvertices.ToArray();
            _underwaterMesh.triangles = utri.ToArray();
            _underwaterMesh.colors = ucolor.ToArray();
            _underwaterMesh.normals = unormals.ToArray();
            _underwaterMesh.RecalculateBounds();
        }
    }

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
        force.force = -(1 - Mathf.Abs(cosTheta)) * viscousDragCoeff * speedAtTriangle * area;

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
        //Vector3 Fd = new Vector3(0, 0, 0);
        //float cosTheta = Vector3.Dot(speedAtTriangle.normalized, normal);
        //float vi = speedAtTriangle.magnitude;
        //float Cd = 0.5f * UnityPhysicsConstants.RHO;
        //float vr = 1;
        //Fd = -Cd * vi / vr * cosTheta * area * normal;
        //force.force = Fd;
        //force.appliPoint = center;


        Vector3 Fd = new Vector3(0, 0, 0);
        float Cpd1 = 1000;
        float Cpd2 = 1000;
        float vi = speedAtTriangle.magnitude;
        float vr = 1;
        float fp = 0.4f;
        float cosTheta = Vector3.Dot(speedAtTriangle.normalized, normal);
        float Csd1 = 1000;
        float Csd2 = 1000;
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

        //Func<int, int> viscousDrag = (i, j) => { };

        foreach (DepthTriangle triangle in _submergedTriangles)
        {
            float area = GeometryTools.ComputeTriangleArea(triangle.p0, triangle.p1, triangle.p2);
            Vector3 center = GeometryTools.ComputeCentroid(triangle.p0, triangle.p1, triangle.p2);
            Vector3 r = center - _rigidbody.worldCenterOfMass;
            Vector3 speedAtTriangle = _rigidbody.velocity + Vector3.Cross(_rigidbody.angularVelocity, r);
            float cosTheta = Vector3.Dot(speedAtTriangle.normalized, triangle.normal);
          
            // Viscous drag (or skin drag) acts on the tangential flow to the surface,
            // so it is stronger for theta ~ 90 deg
            Force viscousForce;
            
            float viscousDragCoeff = 1.0f;           
            viscousForce.force = -(1 - Mathf.Abs(cosTheta)) * viscousDragCoeff * speedAtTriangle * UnityPhysicsConstants.RHO * area;
            viscousForce.appliPoint = center;

            // Pressure force acts orthogonally to the surface and is stronger for theta ~ 0 or 180 deg
            Force pressureForce;

            Vector3 Fd = new Vector3(0, 0, 0);
            float vi = speedAtTriangle.magnitude;
            float Cd = 2.0f;
            float halfRhoS = 0.5f * UnityPhysicsConstants.RHO * area;
            float vr = 1;
            float vel = vi / vr;
            Fd = -halfRhoS * Cd * vel * cosTheta * triangle.normal;
            pressureForce.force = Fd;
            pressureForce.appliPoint = center;

            forces.Add(viscousForce);
            forces.Add(pressureForce);
        }

        return forces;
    }

    List<Force> ComputeHydrodynamics()
    {
        List<Force> forces = new List<Force>();

        foreach (DepthTriangle triangle in _submergedTriangles)
        {
            float area = GeometryTools.ComputeTriangleArea(triangle.p0, triangle.p1, triangle.p2);
            Vector3 center = GeometryTools.ComputeCentroid(triangle.p0, triangle.p1, triangle.p2);
            Vector3 r = center - _rigidbody.worldCenterOfMass;
            Vector3 speedAtTriangle = _rigidbody.velocity + Vector3.Cross(_rigidbody.angularVelocity, r);
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



    List<Force> ComputeBuoyancy(ref bool status)
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
            status = false;
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
            float height0 = worldVertex0.y - verticesHeight[i * 3];
            float height1 = worldVertex1.y - verticesHeight[i * 3 + 1];
            float height2 = worldVertex2.y - verticesHeight[i * 3 + 2];

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

                    Vector3 tmpNormal1 = Vector3.Cross(M.vertex - il, L.vertex - il);
                    Vector3 tmpNormal2 = Vector3.Cross(M.vertex - im, il - im);

                    //Vector3 tmpNormal1 = Vector3.Cross(M.vertex - L.vertex, L.vertex - il);
                    //Vector3 tmpNormal2 = Vector3.Cross(M.vertex - im, il - im);

                    // Ideally, IL and IM should be at the surface. Due to the way we are
                    // performing the cutting, they are obviously not. As an approximation 
                    // and to avoid more water queries, we can assume they are at the surface
                    float heightIl = 0;
                    float heightIm = 0;
                    if (!_approximateMode)
                    {
                        float[] heightCutting = new float[2];
                        _waterProvider.SampleHeightAt(new Vector3[] { il, im }, ref heightCutting);
                        heightIl = il.y - heightCutting[0];
                        heightIm = im.y - heightCutting[1];
                    }

                    DepthTriangle waterTriangle1 = new DepthTriangle(il, M.vertex, L.vertex, heightIl, M.h, L.h, currentTriNormal);
                    DepthTriangle waterTriangle2 = new DepthTriangle(im, M.vertex, il, heightIm, M.h, heightIl, currentTriNormal);

                    if (!CompareNormals(tmpNormal1, currentTriNormal))
                    {
                        waterTriangle1 = new DepthTriangle(il, L.vertex, M.vertex, heightIl, L.h, M.h, currentTriNormal);
                    }
                    if (!CompareNormals(tmpNormal2, currentTriNormal))
                    {
                        waterTriangle2 = new DepthTriangle(im, il, M.vertex, heightIm, heightIl, M.h, currentTriNormal);
                    }

                    _submergedTriangles.Add(waterTriangle1);
                    _submergedTriangles.Add(waterTriangle2);

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

                    float heightJm = 0;
                    float heightJh = 0;
                    if (!_approximateMode)
                    {
                        float[] heightCutting = new float[2];
                        _waterProvider.SampleHeightAt(new Vector3[] { jm, jh }, ref heightCutting);
                        heightJm = jm.y - heightCutting[0];
                        heightJh = jh.y - heightCutting[1];
                    }

                    if (CompareNormals(tmpNormal, currentTriNormal))
                        _submergedTriangles.Add(new DepthTriangle(jh, jm, L.vertex, heightJh, heightJm, L.h, currentTriNormal));
                    else
                        _submergedTriangles.Add(new DepthTriangle(jh, L.vertex, jm, heightJh, L.h, heightJm, currentTriNormal));
                }
            }
            else if (H.h < 0 && M.h < 0 && L.h < 0)
                _submergedTriangles.Add(new DepthTriangle(worldVertex0, worldVertex1, worldVertex2, height0, height1, height2, currentTriNormal));
        }

        List<Force> forces = new List<Force>();

        _submergedSurface = 0;
        foreach (DepthTriangle triangle in _submergedTriangles)
        {
            (VertexAndDepth H, VertexAndDepth M, VertexAndDepth L) = triangle.GetSortedByVertex();
            Debug.Assert(H.vertex.y >= M.vertex.y && H.vertex.y >= L.vertex.y && M.vertex.y >= L.vertex.y);

            float RHO = UnityPhysicsConstants.RHO;
            float absG = -UnityPhysicsConstants.G.y;
            float area = GeometryTools.ComputeTriangleArea(H.vertex, M.vertex, L.vertex);

            _submergedSurface += area;
            Vector3 center = GeometryTools.ComputeCentroid(H.vertex, M.vertex, L.vertex);
            //Debug.Log(center.y);
            float depthCenter;
            if (!_approximateMode)
            {
                float[] heightCenter = new float[1];
                _waterProvider.SampleHeightAt(new Vector3[] { center }, ref heightCenter);
                depthCenter = heightCenter[0] - center.y;
                if (depthCenter < 0)
                {
                    Debug.Log("Depth center : " + depthCenter);
                    continue;
                }
            }
            //else
            //{
            //    // Use barycentric coordinates to estimate the centroid depth
            //    Vector3 e0 = H.vertex - center;
            //    Vector3 e1 = M.vertex - center;
            //    Vector3 e2 = L.vertex - center;

            //    float u = 0.5f * Vector3.Cross(e1, e2).magnitude / area;
            //    float v = 0.5f * Vector3.Cross(e2, e0).magnitude / area;
            //    float w = 1f - u - v;

            //    depthCenter = (float)(u * (-H.h) + v * (-M.h) + w * (-L.h));
            //}

            //Force force = ComputeBuoyancyAtCentroid(H.vertex, M.vertex, L.vertex, triangle.normal, center, depthCenter);      
            //forces.Add(force);

            Force force = ComputeBuoyancyAtCenterOfPressure(H.vertex, M.vertex, L.vertex, (float)H.h, (float)M.h, (float)L.h, triangle.normal);
            forces.Add(force);

            //Debug.DrawLine(force.appliPoint, force.appliPoint + triangle.normal.normalized * 3, Color.magenta);
        }

        //Debug.Log(force.y);

        return forces;
        //return (force: force, torque: torque);
    }



    Force ComputeBuoyancyAtCentroid(Vector3 H, Vector3 M, Vector3 L, Vector3 normal, Vector3 centroid, float depthCentroid)
    {
        Force force;
        float RHO = UnityPhysicsConstants.RHO;
        float absG = Mathf.Abs(UnityPhysicsConstants.G.y);

        float area = GeometryTools.ComputeTriangleArea(H, M, L);

        Vector3 tmpForce = -RHO * absG * area * normal * System.Math.Abs(depthCentroid);
        //Debug.Log(center.y);

        force.force = tmpForce;
        force.appliPoint = centroid;

        return force;
    }

    Force ComputeBuoyancyAtCenterOfPressure(Vector3 H, Vector3 M, Vector3 L, float hH, float hM, float hL, Vector3 normal)
    {
        float RHO = UnityPhysicsConstants.RHO;
        float g = -UnityPhysicsConstants.G.y;

        (Vector3 P, float t) = HorizontalIntersection(H, M, L);
        Vector3 medianPoint = GeometryTools.MidPoint(P, M);

        // Approximation of the height of P / water surface. 
        // Only use if we request approximate height values
        double heightP = MathTools.Interp1(hL, hH, t);// hL * (1 - t) + t * hH;

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
            // Get depth
            float z0 = -(float)hH;
                      
            Vector3 centroidUp = GeometryTools.ComputeCentroid(H, M, P);

            float depthCentroidUp;// = 0;
            if (!_approximateMode)
            {
                float[] waterAtC = new float[1];
                _waterProvider.SampleHeightAt(new Vector3[] { centroidUp }, ref waterAtC);
                depthCentroidUp = waterAtC[0] - centroidUp.y; ;
                //depthCentroidUp = _waterProvider.GetHeightAt(centroidUp) - centroidUp.y;
            }
            else
            {
                //double heightP = hL * (1-t) + t * hH;
                depthCentroidUp = -(float)(1.0 / 3.0 * (hH + hM + heightP));
            }

            if (depthCentroidUp > 0)
            {
                cpUp = ComputeCenterOfPressureApexUp(z0, medianPoint, H);
                float area = GeometryTools.ComputeTriangleArea(H, M, P);
                fUp = -RHO * g * area * System.Math.Abs(depthCentroidUp);
            }
        }

        // If lower base is not horizontal, compute apex down part
        if (M.y > L.y)
        {
            float z0 = 0;
            if(!_approximateMode)
            {
                float[] waterAtMedian = new float[1];
                _waterProvider.SampleHeightAt(new Vector3[] { medianPoint }, ref waterAtMedian);
                z0 = waterAtMedian[0] - medianPoint.y;
                //float z0 = _waterProvider.GetHeightAt(medianPoint) - medianPoint.y;
            }
            else
            {
                //double heightP = (hL * (1 - t) + t * hH);
                z0 = -(float)(0.5 * (heightP + hM));
            }
            
            Vector3 centroidDown = GeometryTools.ComputeCentroid(L, M, P);

            float depthCentroidDown = 0;
            if (!_approximateMode)
            {
                float[] waterAtC = new float[1];
                _waterProvider.SampleHeightAt(new Vector3[] { centroidDown }, ref waterAtC);
                depthCentroidDown = waterAtC[0] - centroidDown.y;
                //float depthCentroidDown = _waterProvider.GetHeightAt(centroidDown) - centroidDown.y;
            }
            else
            {
                //double heightP = (hL * (1-t) + t * hH);
                depthCentroidDown = -(float)(1.0 / 3.0 * (hL + hM + heightP));
            }
            
            if (depthCentroidDown > 0)
            {
                cpDown = ComputeCenterOfPressureApexDown(z0, medianPoint, L);
                float area = GeometryTools.ComputeTriangleArea(L, M, P);
                fDown = -RHO * g * area * System.Math.Abs(depthCentroidDown);
            }
        }

        Force force;
        force.force = normal * (fUp + fDown);
        force.appliPoint = GeometryTools.ComputeBarycenter(fUp, cpUp, fDown, cpDown);

        return force;
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

    (Vector3, float) HorizontalIntersection(Vector3 H, Vector3 M, Vector3 L)
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

        return 
            (new Vector3(
                L.x + (H.x - L.x) * t,
                M.y,
                L.z + (H.z - L.z) * t),
             t);
    }

    bool CompareNormals(Vector3 a, Vector3 b)
    {
        if (Mathf.Sign(a.x) == Mathf.Sign(b.x) && Mathf.Sign(a.y) == Mathf.Sign(b.y) && Mathf.Sign(a.z) == Mathf.Sign(b.z))
            return true;
        return false;
    }
}

