using System.Collections.Generic;
using UnityEngine;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;

namespace ClothSimulation
{
    public class MassSpring : MonoBehaviour, ISimulable
    {
        public MassSpring()
        {
            Manager = null;
        }

        #region EditorVariables

        [Header("Visualization")]
        [SerializeField] private float nodeSize = 0.1f;
        [SerializeField] private float springWidth = 0.1f;
        [Header("Simulation")]
        public float Mass;
        public float StiffnessStretch;
        public float StiffnessBend;
        public float DampingAlpha;
        public float DampingBeta;

        #endregion

        #region OtherVariables
        private PhysicsManager Manager = new();
        private List<Node> Nodes = new();
        private List<Spring> Springs = new();
        private int index;
        private Mesh mesh;
        private Vector3[] vertices;
        private int[] triangles;
        #endregion

        #region MonoBehaviour

        private void Awake()
        {
            mesh = this.GetComponent<MeshFilter>().mesh;
            vertices = mesh.vertices;
            triangles = mesh.triangles;
        }

        public void Update()
        {
            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i] = transform.InverseTransformPoint(Nodes[i].Pos);
            }
            mesh.vertices = vertices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
        }
        #endregion

        #region ISimulable

        public void Initialize(int ind, PhysicsManager m, List<Fixer> fixers)
        {
            Manager = m;
            index = ind;
            //System creation:
            foreach (var v in vertices)
            {
                CreateNode(transform.TransformPoint(v), fixers);
            }
            CreateStretchSprings(triangles);
            CreateBendSprings();
            //System initialization:
            for (int i = 0; i < Nodes.Count; ++i)
                Nodes[i].Initialize(index + 3 * i, Mass / Nodes.Count, DampingAlpha, Manager);

            for (int i = 0; i < Springs.Count; ++i)
            {
                float stiffness = Springs[i].springType == Spring.SpringType.Stretch ? StiffnessStretch : StiffnessBend;
                Color color = Springs[i].springType == Spring.SpringType.Stretch ? Color.cyan : Color.green;
                Springs[i].Initialize(stiffness, DampingBeta, Manager, color);
            }
        }

        public int GetNumDoFs()
        {
            return 3 * Nodes.Count;
        }

        public void GetPosition(VectorXD position)
        {
            for (int i = 0; i < Nodes.Count; ++i)
                Nodes[i].GetPosition(position);
        }

        public void SetPosition(VectorXD position)
        {
            for (int i = 0; i < Nodes.Count; ++i)
                Nodes[i].SetPosition(position);
            for (int i = 0; i < Springs.Count; ++i)
                Springs[i].UpdateState();
        }

        public void GetVelocity(VectorXD velocity)
        {
            for (int i = 0; i < Nodes.Count; ++i)
                Nodes[i].GetVelocity(velocity);
        }

        public void SetVelocity(VectorXD velocity)
        {
            for (int i = 0; i < Nodes.Count; ++i)
                Nodes[i].SetVelocity(velocity);
        }

        public void GetForce(VectorXD force)
        {
            for (int i = 0; i < Nodes.Count; ++i)
                Nodes[i].GetForce(force);
            for (int i = 0; i < Springs.Count; ++i)
                Springs[i].GetForce(force);
        }

        public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
        {
            for (int i = 0; i < Nodes.Count; ++i)
                Nodes[i].GetForceJacobian(dFdx, dFdv);
            for (int i = 0; i < Springs.Count; ++i)
                Springs[i].GetForceJacobian(dFdx, dFdv);
        }

        public void GetMass(MatrixXD mass)
        {
            for (int i = 0; i < Nodes.Count; ++i)
                Nodes[i].GetMass(mass);
        }

        public void GetMassInverse(MatrixXD massInv)
        {
            for (int i = 0; i < Nodes.Count; ++i)
                Nodes[i].GetMassInverse(massInv);
        }

        public void FixVector(VectorXD v)
        {
            for (int i = 0; i < Nodes.Count; i++)
            {
                Nodes[i].FixVector(v);
            }
        }

        public void FixMatrix(MatrixXD M)
        {
            for (int i = 0; i < Nodes.Count; i++)
            {
                Nodes[i].FixMatrix(M);
            }
        }

        #endregion

        #region OtherMethods
        private void CreateNode(Vector3 pos, List<Fixer> fixers)
        {
            GameObject nodeGO = new();
            nodeGO.AddComponent<MeshFilter>().mesh = Resources.GetBuiltinResource<Mesh>("Sphere.fbx");
            nodeGO.AddComponent<MeshRenderer>().material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            Node n = nodeGO.AddComponent<Node>();
            n.Init(pos);
            nodeGO.transform.localPosition = pos;
            nodeGO.transform.localScale = new(nodeSize, nodeSize, nodeSize);
            nodeGO.transform.parent = this.transform;
            nodeGO.name = "Node";
            foreach (Fixer f in fixers) if (f.IsInside(pos)) n.Fixed = true;
            Nodes.Add(n);
        }

        private void CreateStretchSprings(int[] triangles)
        {
            for (int i = 0; i < triangles.Length; i += 3)
            {
                int a = triangles[i];
                int b = triangles[i + 1];
                int c = triangles[i + 2];
                CreateSingleSpring(a, b, c);
                CreateSingleSpring(b, c, a);
                CreateSingleSpring(c, a, b);
            }
        }

        private void CreateBendSprings()
        {
            (int, int)[] keys = new (int, int)[opVertex.Count];
            opVertex.Keys.CopyTo(keys, 0);
            foreach (var halfEdge in keys)
            {
                var otherHalfEdge = (halfEdge.Item2, halfEdge.Item1);
                if (opVertex.ContainsKey(otherHalfEdge))
                {
                    CreateSingleSpring(opVertex[halfEdge], opVertex[otherHalfEdge], -1);
                }
            }
        }

        readonly Dictionary<(int, int), int> opVertex = new(); //This dictionary stores the opposite vertex of each edge
        private void CreateSingleSpring(int idx1, int idx2, int op)
        {
            var halfEdge1 = (idx1, idx2);
            var halfEdge2 = (idx2, idx1);
            bool springCreated = opVertex.ContainsKey(halfEdge1) || opVertex.ContainsKey(halfEdge2);
            opVertex[(idx1, idx2)] = op;
            if (springCreated) return;
            Node nodeA = Nodes[idx1];
            Node nodeB = Nodes[idx2];
            GameObject spring = new();
            LineRenderer line = spring.AddComponent<LineRenderer>();
            line.material = new Material(Shader.Find("Sprites/Default"));
            line.startWidth = springWidth;
            line.endWidth = springWidth;
            line.SetPosition(0, nodeA.Pos);
            line.SetPosition(1, nodeB.Pos);
            Spring s = spring.AddComponent<Spring>();
            s.Init(nodeA, nodeB, op != -1 ? Spring.SpringType.Stretch : Spring.SpringType.Bend);
            spring.transform.SetParent(this.transform);
            spring.name = "Spring";
            Springs.Add(s);
        }
        #endregion
    }
}