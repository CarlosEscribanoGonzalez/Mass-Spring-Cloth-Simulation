using UnityEngine;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

namespace ClothSimulation
{
    public class PhysicsManager : MonoBehaviour
    {
        public enum Integration
        {
            Explicit = 0,
            Symplectic = 1,
            Implicit = 2,
        };

        #region InEditorVariables

        public bool Paused = true;
        public float TimeStep = 0.01f;
        public Vector3 Gravity = new(0.0f, -9.81f, 0.0f);
        public Integration IntegrationMethod = Integration.Explicit;
        public List<GameObject> SimObjects;
        public List<Fixer> Fixers;

        #endregion

        #region OtherVariables
        private List<ISimulable> m_objs;
        private int m_numDoFs;
        private VectorXD x;
        private VectorXD v;
        private VectorXD f;
        private MatrixXD Minv;
        private MatrixXD M;
        private MatrixXD posJacobian;
        private MatrixXD velJacobian;
        private DenseMatrixXD I;
        #endregion

        #region MonoBehaviour

        public void Start()
        {
            Time.fixedDeltaTime = IntegrationMethod == Integration.Implicit ? 0.1f : Mathf.Clamp(TimeStep, 0.01f, 0.2f);
            //Parse the simulable objects and initialize their state indices
            m_numDoFs = 0;
            m_objs = new List<ISimulable>(SimObjects.Capacity);

            foreach (GameObject obj in SimObjects)
            {
                ISimulable simobj = obj.GetComponent<ISimulable>();
                if (simobj != null)
                {
                    m_objs.Add(simobj);

                    //Initialize simulable object
                    simobj.Initialize(m_numDoFs, this, Fixers);

                    //Retrieve pos and vel size
                    m_numDoFs += simobj.GetNumDoFs();
                }
            }

            x = new DenseVectorXD(m_numDoFs);
            v = new DenseVectorXD(m_numDoFs);
            f = new DenseVectorXD(m_numDoFs);
            Minv = new DenseMatrixXD(m_numDoFs);
            M = new DenseMatrixXD(m_numDoFs);
            posJacobian = new DenseMatrixXD(m_numDoFs);
            velJacobian = new DenseMatrixXD(m_numDoFs);
            I = DenseMatrixXD.CreateIdentity(m_numDoFs);
        }

        public void Update()
        {
            if (Input.GetKeyUp(KeyCode.P))
                this.Paused = !this.Paused;
        }

        public void FixedUpdate()
        {
            if (this.Paused)
                return; //Not simulating

            //Select integration method
            switch (this.IntegrationMethod)
            {
                case Integration.Explicit: this.StepExplicit(); break;
                case Integration.Symplectic: this.StepSymplectic(); break;
                case Integration.Implicit: this.StepImplicit(); break;
                default:
                    throw new System.Exception("[ERROR] Should never happen!");
            }
        }

        #endregion

        private void GetVariables(bool getJacobian = false)
        {
            x.Clear();
            v.Clear();
            f.Clear();
            if (getJacobian)
            {
                posJacobian.Clear();
                velJacobian.Clear();
            }
            foreach (ISimulable obj in m_objs)
            {
                obj.GetPosition(x);
                obj.GetVelocity(v);
                obj.GetForce(f);
                obj.GetMassInverse(Minv);
                obj.GetMass(M);
                if (getJacobian) obj.GetForceJacobian(posJacobian, velJacobian);
            }
            foreach (ISimulable obj in m_objs)
            {
                obj.FixVector(f);
                obj.FixMatrix(Minv);
                obj.FixMatrix(M);
            }
        }

        private void UpdateSimulation()
        {
            foreach (ISimulable obj in m_objs)
            {
                obj.SetPosition(x);
                obj.SetVelocity(v);
            }
        }

        private void StepExplicit()
        {
            GetVariables();
            x += TimeStep * v;
            v += TimeStep * (Minv * f);
            UpdateSimulation();
        }

        private void StepSymplectic()
        {
            GetVariables();
            v += TimeStep * (Minv * f);
            x += TimeStep * v;
            UpdateSimulation();
        }

        private void StepImplicit()
        {
            GetVariables(true);
            MatrixXD A = M - TimeStep * velJacobian - TimeStep * TimeStep * posJacobian;
            VectorXD b = (M - TimeStep * velJacobian) * v + TimeStep * f;
            foreach (ISimulable obj in m_objs)
            {
                obj.FixMatrix(A);
                obj.FixVector(b);
            }
            v = A.Solve(b);
            x += TimeStep * v;
            UpdateSimulation();
        }
    }
}