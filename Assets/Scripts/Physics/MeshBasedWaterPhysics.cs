﻿using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using System.Linq;

namespace Assets.Scripts.Physics
{
    public class MeshBasedWaterPhysics : MonoBehaviour
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
            public DepthTriangle(Vector3 p1, Vector3 p2, Vector3 p3, double depth1, double depth2, double depth3, Vector3 normal, int index)
            {
                vertices = new VertexAndDepth[3];
                vertices[0] = new VertexAndDepth(p1, depth1);
                vertices[1] = new VertexAndDepth(p2, depth2);
                vertices[2] = new VertexAndDepth(p3, depth3);

                //_oldIndices = new int[] { 0, 1, 2 };
                //Array.Sort(vertices, _oldIndices, new SortedVector3Comparer());

                this.normal = normal;

                surfaceTriIndex = index;

                _slamming = 0;
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

            public float Slamming { get => _slamming; set => _slamming = value; }

            public Vector3 normal;

            private VertexAndDepth[] vertices;

            public int surfaceTriIndex;

            private float _slamming;
        }

        private DepthTriangle[] _trianglesAndVertices;
        private Mesh _mesh;
        List<DepthTriangle> _submergedTriangles;
        List<DepthTriangle> _emergedTriangles;

        Vector3[] _worldVertices;
        float[] _waterHeight;

        public GameObject _underwaterGameObject;
        Mesh _underwaterMesh;

        float _meshArea = 0;
        float _submergedArea = 0;

        IWaterProvider _waterProvider;

        Body _body;
        BodyState _bodyState;

        public float _mass = 100;
        public Vector3 _cogOffset = new Vector3(0, 0, 0);
        public Vector3 _diagInertia = new Vector3(50, 50, 50);

        public RefFrame _refFrame = RefFrame.WORLD;

        protected Rigidbody _rigidbody;


        public bool _isDebug = true;

        public bool _approximateMode = true;

        private Vector3[] _speedAtOrigTriangle;
        private Vector3[] _lastSpeedAtOrigTriangle;
        private float[] _lastOrigArea;
        private float[] _origArea;

        // Hydrodynamics
        public float _Cpd1 = 100;
        public float _Cpd2 = 100;
        public float _vr = 1;
        public float _fp = 0.4f;
        public float _Csd1 = 100;
        public float _Csd2 = 100;
        public float _fs = 0.4f;

        public int _pSlamming = 1;

        public bool _computeViscous = true;
        public bool _computePressure = true;
        public bool _computeSlamming = true;


        // REMOVE
        //Crest.SampleHeightHelper _samplerTest;

        public float MeshArea { get => _meshArea; set => _meshArea = value; }
        public float SubmergedArea { get => _submergedArea; set => _submergedArea = value; }

        // Start is called before the first frame update
        void Start()
        {
            //Unity.Jobs.IJobParallelFor f;

            _mesh = GetComponent<MeshFilter>().sharedMesh;
            _trianglesAndVertices = new DepthTriangle[_mesh.triangles.Length / 3];
            _submergedTriangles = new List<DepthTriangle>();

            this.transform.GetChild(0).GetComponent<MeshFilter>().name = "underwater";
            _underwaterMesh = this.transform.GetChild(0).GetComponent<MeshFilter>().mesh;

            _speedAtOrigTriangle = new Vector3[_trianglesAndVertices.Length];
            _lastSpeedAtOrigTriangle = new Vector3[_trianglesAndVertices.Length];
            _lastOrigArea = new float[_trianglesAndVertices.Length];
            _origArea = new float[_trianglesAndVertices.Length];
            
            _worldVertices = new Vector3[_trianglesAndVertices.Length * 3];
            _waterHeight = new float[_worldVertices.Length];

            Vector3[] vertices = _mesh.vertices;
            for (int i = 0; i < _trianglesAndVertices.Length; i++)
            {
                int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
                Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
                Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
                Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

                MeshArea += GeometryTools.ComputeTriangleArea(worldVertex0, worldVertex1, worldVertex2);

                _speedAtOrigTriangle[i] = new Vector3(0, 0, 0);
                _lastSpeedAtOrigTriangle[i] = new Vector3(0, 0, 0);
                _lastOrigArea[i] = 0;
                _origArea[i] = 0;
            }


            GameObject[] oceanGO = GameObject.FindGameObjectsWithTag("Ocean");
            _waterProvider = oceanGO[0].GetComponent<IWaterProvider>();
            
            // TODO
            //_samplerTest = new Crest.SampleHeightHelper();


            //
            _rigidbody = GetComponent<Rigidbody>();
            _rigidbody.useGravity = false;
            _rigidbody.detectCollisions = false;
            _rigidbody.inertiaTensor = _diagInertia;
            _rigidbody.mass = _mass;
            _rigidbody.centerOfMass = _cogOffset;
        }

        void FixedUpdate()
        {
            List<Force> allForces = new List<Force>();
            Vector3 sumForces = new Vector3();

            Force fweight = ComputeWeight();
            allForces.Add(fweight);

            bool status = true;
            //List<Force> forcesB = 
            ComputeWaterPhysics(ref status);
            //if (!status)
            //    return;
            //allForces.AddRange(forcesB);

            //if (_computeHydrodynamics)
            //{
            //    //allForces.AddRange(ComputeEmpiricalDrag());
            //    allForces.AddRange(ComputeHydrodynamics());
            //}

            //if (allForces[allForces.Count - 1].force.y > 100000)
            //    Debug.Log(allForces[allForces.Count - 1].force.y);

            //foreach (Force fb in allForces)
            //{
            //    if (_refFrame == RefFrame.WORLD)
            //        _rigidbody.AddForceAtPosition(fb.force, fb.appliPoint);
            //    sumForces += fb.force;
            //}

            //ClearDebugMesh();
            //if (_isDebug)
            //{
            //    ShowDebugMesh();
            //}
        }

        Force ComputeWeight()
        {
            Force force = new Force();
            Vector3 torque = new Vector3();

            force.force = _mass * UnityPhysicsConstants.G;
            force.appliPoint = _rigidbody.worldCenterOfMass;// this.transform.position;

            //Debug.Log(force.y);

            return force;
        }


       void ComputeWaterPhysics(ref bool status)
        {
            Vector3[] vertices = _mesh.vertices;

            _submergedTriangles.Clear();

            for (int i = 0; i < _trianglesAndVertices.Length; i++)
            {
                int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };
                Vector3 worldVertex0 = transform.TransformPoint(vertices[indices[0]]);
                Vector3 worldVertex1 = transform.TransformPoint(vertices[indices[1]]);
                Vector3 worldVertex2 = transform.TransformPoint(vertices[indices[2]]);

                _worldVertices[i * 3] = worldVertex0;
                _worldVertices[i * 3 + 1] = worldVertex1;
                _worldVertices[i * 3 + 2] = worldVertex2;
            }

            //float[] verticesHeight = new float[_worldVertices.Length];
            if (!_waterProvider.SampleHeightAt(_worldVertices, ref _waterHeight))
            {
                Debug.LogError("Sampling error");
                status = false;
                return;// new List<Force>();
            }

            int nbThreads = 8;
            Thread[] threads = new Thread[nbThreads];
            int step = _trianglesAndVertices.Length / nbThreads;
            for (int i = 0; i < nbThreads; ++i)
            {
                int[] ind = {i * step, (i + 1) * step};

                threads[i] = new Thread(new ParameterizedThreadStart(WaterPhysicsFunc));

                threads[i].Start(ind);
            }
        }

        void WaterPhysicsFunc(object threadIndices)//int minIndex, int maxIndex)
        {
            int[] unfoldedIndices = (int[]) threadIndices;
            for (int i = unfoldedIndices[0]; i < unfoldedIndices[1]; i++)
            {
                Vector3 worldVertex0 = _worldVertices[i * 3];
                Vector3 worldVertex1 = _worldVertices[i * 3 + 1];
                Vector3 worldVertex2 = _worldVertices[i * 3 + 2];

                //int[] indices = { _mesh.triangles[i * 3], _mesh.triangles[(i * 3) + 1], _mesh.triangles[(i * 3) + 2] };

                Vector3 center = GeometryTools.ComputeCentroid(worldVertex0, worldVertex1, worldVertex2);
                Vector3 r = center - _rigidbody.worldCenterOfMass;
                Vector3 speedAtTriangle = _rigidbody.velocity + Vector3.Cross(_rigidbody.angularVelocity, r);
                _lastSpeedAtOrigTriangle[i] = _speedAtOrigTriangle[i];
                _speedAtOrigTriangle[i] = speedAtTriangle;
                _lastOrigArea[i] = _origArea[i];

                // Height relative to surface water. Negative values mean underwater vertices
                float height0 = worldVertex0.y - _waterHeight[i * 3];
                float height1 = worldVertex1.y - _waterHeight[i * 3 + 1];
                float height2 = worldVertex2.y - _waterHeight[i * 3 + 2];

                Vector3 geometricNormal = GeometryTools.ComputeNormal(worldVertex0, worldVertex1, worldVertex2);

                //Vector3 currentTriNormal = (_mesh.normals[indices[0]] + _mesh.normals[indices[1]] + _mesh.normals[indices[2]]) / 3.0f;
                ////should take care of the scaling factor...
                //currentTriNormal = transform.TransformDirection(currentTriNormal);
                //currentTriNormal.Normalize();
                //if (!CompareNormals(currentTriNormal, geometricNormal))
                //    Debug.LogWarning("Incoherent normals " + currentTriNormal + " " + geometricNormal);

                DepthTriangle triangle = new DepthTriangle(worldVertex0, worldVertex1, worldVertex2,
                                                           height0, height1, height2,
                                                           geometricNormal,
                                                           i);
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

                        Vector3 tmpNormal1 = Vector3.Cross(M.vertex - il, L.vertex - il).normalized;
                        Vector3 tmpNormal2 = Vector3.Cross(M.vertex - im, il - im).normalized;

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

                        DepthTriangle waterTriangle1 = new DepthTriangle(il, M.vertex, L.vertex, heightIl, M.h, L.h, tmpNormal1, i);
                        DepthTriangle waterTriangle2 = new DepthTriangle(im, M.vertex, il, heightIm, M.h, heightIl, tmpNormal2, i);

                        if (!CompareNormals(tmpNormal1, geometricNormal))
                        {
                            waterTriangle1 = new DepthTriangle(il, L.vertex, M.vertex, heightIl, L.h, M.h, -tmpNormal1, i);
                        }
                        if (!CompareNormals(tmpNormal2, geometricNormal))
                        {
                            waterTriangle2 = new DepthTriangle(im, il, M.vertex, heightIm, heightIl, M.h, -tmpNormal2, i);
                        }

                        _submergedTriangles.Add(waterTriangle1);
                        _submergedTriangles.Add(waterTriangle2);

                        _origArea[i] = GeometryTools.ComputeTriangleArea(waterTriangle1.p0, waterTriangle1.p1, waterTriangle1.p2) +
                                       GeometryTools.ComputeTriangleArea(waterTriangle2.p0, waterTriangle2.p1, waterTriangle2.p2);

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

                        Vector3 tmpNormal = Vector3.Cross(jm - jh, L.vertex - jh).normalized;

                        float heightJm = 0;
                        float heightJh = 0;
                        if (!_approximateMode)
                        {
                            float[] heightCutting = new float[2];
                            _waterProvider.SampleHeightAt(new Vector3[] { jm, jh }, ref heightCutting);
                            heightJm = jm.y - heightCutting[0];
                            heightJh = jh.y - heightCutting[1];
                        }

                        if (CompareNormals(tmpNormal, geometricNormal))
                            _submergedTriangles.Add(new DepthTriangle(jh, jm, L.vertex, heightJh, heightJm, L.h, tmpNormal, i));
                        else
                            _submergedTriangles.Add(new DepthTriangle(jh, L.vertex, jm, heightJh, L.h, heightJm, -tmpNormal, i));

                        _origArea[i] = GeometryTools.ComputeTriangleArea(_submergedTriangles.Last().p0, _submergedTriangles.Last().p1, _submergedTriangles.Last().p2);
                    }
                }
                else if (H.h < 0 && M.h < 0 && L.h < 0)
                {
                    _submergedTriangles.Add(new DepthTriangle(worldVertex0, worldVertex1, worldVertex2, height0, height1, height2, geometricNormal, i));

                    _origArea[i] = GeometryTools.ComputeTriangleArea(_submergedTriangles.Last().p0, _submergedTriangles.Last().p1, _submergedTriangles.Last().p2);
                }
            }

            List<Force> forces = new List<Force>();

            _submergedArea = 0;
            foreach (DepthTriangle triangle in _submergedTriangles)
            {
                (VertexAndDepth H, VertexAndDepth M, VertexAndDepth L) = triangle.GetSortedByVertex();
                Debug.Assert(H.vertex.y >= M.vertex.y && H.vertex.y >= L.vertex.y && M.vertex.y >= L.vertex.y);

                //float RHO = UnityPhysicsConstants.RHO;
                //float absG = -UnityPhysicsConstants.G.y;
                float area = GeometryTools.ComputeTriangleArea(H.vertex, M.vertex, L.vertex);

                _submergedArea += area;
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

            foreach (Force force in forces)
                _rigidbody.AddForceAtPosition(force.force, force.appliPoint);

            //return forces;
            //return (force: force, torque: torque);
        }


        /// <summary>
        /// Computes the buoyancy force at the triangle's centroid
        /// </summary>
        /// <param name="H"></param>
        /// <param name="M"></param>
        /// <param name="L"></param>
        /// <param name="normal"></param>
        /// <param name="centroid"></param>
        /// <param name="depthCentroid"></param>
        /// <returns></returns>
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

        /// <summary>
        /// Computes the buoyancy force at the triangle's center of pressure
        /// </summary>
        /// <param name="H"></param>
        /// <param name="M"></param>
        /// <param name="L"></param>
        /// <param name="hH"></param>
        /// <param name="hM"></param>
        /// <param name="hL"></param>
        /// <param name="normal"></param>
        /// <returns></returns>
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
                if (!_approximateMode)
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
            return (Vector3.Dot(a, b) >= 0);
        }
    }
}