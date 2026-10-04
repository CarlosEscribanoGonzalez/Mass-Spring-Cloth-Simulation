using UnityEngine;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

namespace ClothSimulation
{
    public class Spring : MonoBehaviour
    {

        #region InEditorVariables

        public float Stiffness;
        public float Damping;
        public Node nodeA;
        public Node nodeB;

        #endregion

        public enum SpringType { Stretch, Bend };
        public SpringType springType;

        public float Length0;
        public float Length;

        private LineRenderer lineRenderer;
        private Vector3 springForce = Vector3.zero;
        private MatrixXD I;  //Identity matrix used to compute the jacobian
        private Vector3 dir; //Vector director

        public void Init(Node a, Node b, SpringType s)
        {
            nodeA = a;
            nodeB = b;
            springType = s;
        }

        //Use this for initialization
        public void Initialize(float stiffness, float damping, PhysicsManager m, Color springColor)
        {
            lineRenderer = GetComponent<LineRenderer>();
            Stiffness = stiffness;
            Damping = damping;
            UpdateState();
            Length0 = Length;
            I = DenseMatrixXD.CreateIdentity(3);
            lineRenderer.startColor = lineRenderer.endColor = springColor;
        }

        //Update spring state
        public void UpdateState()
        {
            dir = nodeA.Pos - nodeB.Pos;
            Length = dir.magnitude;
            dir = (1.0f / Length) * dir;
            lineRenderer.SetPosition(0, nodeA.Pos);
            lineRenderer.SetPosition(1, nodeB.Pos);
        }

        public void GetForce(VectorXD force)
        {
            springForce = -Stiffness * (Length - Length0) * dir;
            springForce -= Damping * Stiffness * Vector3.Dot(nodeA.Vel - nodeB.Vel, dir) * dir;
            int indexA = nodeA.index;
            force[indexA] += springForce.x;
            force[indexA + 1] += springForce.y;
            force[indexA + 2] += springForce.z;
            int indexB = nodeB.index;
            force[indexB] -= springForce.x;
            force[indexB + 1] -= springForce.y;
            force[indexB + 2] -= springForce.z;
        }

        VectorXD u = new DenseVectorXD(3); //dir but as a dense vector
        public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
        {
            int indexA = nodeA.index;
            int indexB = nodeB.index;
            u[0] = dir.x;
            u[1] = dir.y;
            u[2] = dir.z;
            MatrixXD uut = u.ToColumnMatrix() * u.ToRowMatrix();
            //Jacobian of force with respect to position:
            MatrixXD forceDiff = -Stiffness * (Length - Length0) / Length *
                (I - uut) - Stiffness * uut;
            FillMatrix(dFdx, indexA, indexA, forceDiff);
            FillMatrix(dFdx, indexB, indexB, forceDiff);
            FillMatrix(dFdx, indexA, indexB, -forceDiff);
            FillMatrix(dFdx, indexB, indexA, -forceDiff);
            //Jacobian of force with respect to velocity:
            forceDiff = -Damping * Stiffness * uut;
            FillMatrix(dFdv, indexA, indexA, forceDiff);
            FillMatrix(dFdv, indexB, indexB, forceDiff);
            FillMatrix(dFdv, indexA, indexB, -forceDiff);
            FillMatrix(dFdv, indexB, indexA, -forceDiff);
        }

        public void FillMatrix(MatrixXD dFdx, int forceIndex, int posIndex, MatrixXD forceDiff)
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    dFdx[forceIndex + j, posIndex + i] += forceDiff[j, i];
                }
            }
        }
    }
}