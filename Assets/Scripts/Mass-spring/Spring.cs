using UnityEngine;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;

namespace MassSpring
{
    public class Spring : MonoBehaviour
    {

        #region InEditorVariables

        public float Stiffness;
        public Node nodeA;
        public Node nodeB;

        #endregion

        public float Length0;
        public float Length;

        private Vector3 springForce = Vector3.zero;
        private PhysicsManager Manager;

        private MatrixXD I; //Identity matrix used to compute the jacobian
        private Vector3 u; //Vector director

        // Update is called once per frame
        void Update()
        {
            Vector3 yaxis = new Vector3(0.0f, 1.0f, 0.0f);
            Vector3 dir = nodeA.Pos - nodeB.Pos;
            dir.Normalize();

            transform.position = 0.5f * (nodeA.Pos + nodeB.Pos);
            //The default length of a cylinder in Unity is 2.0
            transform.localScale = new Vector3(transform.localScale.x, Length / 2.0f, transform.localScale.z);
            transform.rotation = Quaternion.FromToRotation(yaxis, dir);
        }

        // Use this for initialization
        public void Initialize(PhysicsManager m)
        {
            Manager = m;

            UpdateState();
            Length0 = Length;
            I = DenseMatrixXD.CreateIdentity(3);
        }

        //Update spring state
        public void UpdateState()
        {
            Length = (nodeA.Pos - nodeB.Pos).magnitude;
            u = (nodeA.Pos - nodeB.Pos) / Length;
        }

        //Get Force
        public void GetForce(VectorXD force)
        {
            springForce = -Stiffness * (Length - Length0) * u;
            int indexA = nodeA.index;
            force[indexA] += springForce.x;
            force[indexA + 1] += springForce.y;
            force[indexA + 2] += springForce.z;
            int indexB = nodeB.index;
            force[indexB] -= springForce.x;
            force[indexB + 1] -= springForce.y;
            force[indexB + 2] -= springForce.z;
        }

        VectorXD uVector = new DenseVectorXD(3);
        public void GetForceJacobian(MatrixXD dFdx)
        {
            int indexA = nodeA.index;
            int indexB = nodeB.index;
            uVector[0] = u.x;
            uVector[1] = u.y;
            uVector[2] = u.z;
            MatrixXD uut = uVector.ToColumnMatrix() * uVector.ToRowMatrix();
            //Variation in force as the position at A changes:
            MatrixXD forceDiff = -Stiffness * (Length - Length0) / Length *
                (I - uut) - Stiffness * uut;
            FillMatrix(dFdx, indexA, indexA, forceDiff);
            FillMatrix(dFdx, indexB, indexB, forceDiff);
            FillMatrix(dFdx, indexA, indexB, -forceDiff);
            FillMatrix(dFdx, indexB, indexA, -forceDiff);
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