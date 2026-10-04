using UnityEngine;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;
using System.Security.Principal;

namespace MassSpring
{
    public class PhysicsManager : MonoBehaviour
    {
        public PhysicsManager()
        {
            Paused = true;
            TimeStep = 0.01f;
            Gravity = new Vector3(0.0f, -9.81f, 0.0f);
            IntegrationMethod = Integration.Explicit;
        }

        public enum Integration
        {
            Explicit = 0,
            Symplectic = 1,
            Midpoint = 2,
            Verlet = 3,
            Implicit = 4,
        };

        #region InEditorVariables

        public bool Paused;
        public float TimeStep;
        public Vector3 Gravity;
        public List<GameObject> SimObjects;
        public Integration IntegrationMethod;

        #endregion

        #region OtherVariables
        private List<ISimulable> m_objs;
        private int m_numDoFs;
        private VectorXD x;
        private VectorXD xPrev; //prev x for Verlet
        private VectorXD v;
        private VectorXD f;
        private MatrixXD Minv;
        private MatrixXD M;
        private MatrixXD forceJacobian;
        private DenseMatrixXD I;
        #endregion

        #region MonoBehaviour

        public void Start()
        {
            Time.fixedDeltaTime = Mathf.Clamp(TimeStep, 0.01f, 0.2f);
            //Parse the simulable objects and initialize their state indices
            m_numDoFs = 0;
            m_objs = new List<ISimulable>(SimObjects.Capacity);

            foreach (GameObject obj in SimObjects)
            {
                ISimulable simobj = obj.GetComponent<ISimulable>();
                if (simobj != null)
                {
                    m_objs.Add(simobj);

                    // Initialize simulable object
                    simobj.Initialize(m_numDoFs, this);

                    // Retrieve pos and vel size
                    m_numDoFs += simobj.GetNumDoFs();
                }
            }

            x = new DenseVectorXD(m_numDoFs);
            xPrev = new DenseVectorXD(m_numDoFs);
            v = new DenseVectorXD(m_numDoFs);
            f = new DenseVectorXD(m_numDoFs);
            Minv = new DenseMatrixXD(m_numDoFs);
            M = new DenseMatrixXD(m_numDoFs);
            forceJacobian = new DenseMatrixXD(m_numDoFs);
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
                return; // Not simulating

            // Select integration method
            switch (this.IntegrationMethod)
            {
                case Integration.Explicit: this.StepExplicit(); break;
                case Integration.Symplectic: this.StepSymplectic(); break;
                case Integration.Midpoint: this.StepMidpoint(); break;
                case Integration.Verlet: this.StepVerlet(); break;
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
            if (getJacobian) forceJacobian.Clear();
            foreach (ISimulable obj in m_objs)
            {
                obj.GetPosition(x);
                obj.GetVelocity(v);
                obj.GetForce(f);
                obj.GetMassInverse(Minv);
                if (getJacobian)
                {
                    obj.GetForceJacobian(forceJacobian);
                    obj.GetMass(M);
                }
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
            //Now velocities are calculated before positions
            v += TimeStep * (Minv * f);
            x += TimeStep * v;
            UpdateSimulation();
        }

        private void StepMidpoint()
        {
            GetVariables();
            //Reference to the init values:
            VectorXD initX = x.Clone();
            VectorXD initV = v.Clone();
            //Midpoints:
            v += TimeStep * 0.5f * (Minv * f); //v first so it's more stable
            x += TimeStep * 0.5f * v;
            //Midpoint update:
            UpdateSimulation();
            GetVariables();
            //Compute final values:
            v = initV + TimeStep * Minv * f;
            x = initX + TimeStep * v;
            UpdateSimulation();
        }

        bool firstStep = true;
        private void StepVerlet()
        {
            GetVariables();
            VectorXD initX = x.Clone();
            if (firstStep) //First step, without xPrev
            {
                x += v * TimeStep + 0.5 * Minv * f * TimeStep * TimeStep;
                firstStep = false;
            }
            else x = 2 * x - xPrev + (Minv * f) * TimeStep * TimeStep;
            v = (x - xPrev) / (2 * TimeStep);
            initX.CopyTo(xPrev);
            UpdateSimulation();
        }

        private void StepImplicit()
        {
            GetVariables(true); //true param allows the method to update forceJacobian
            MatrixXD a = M - TimeStep * TimeStep * forceJacobian;
            VectorXD b = M * v + TimeStep * f;
            foreach (ISimulable obj in m_objs)
            {
                obj.FixMatrix(a);
                obj.FixVector(b);
            }
            v = a.Solve(b);
            x += TimeStep * v;
            UpdateSimulation();
        }
    }
}