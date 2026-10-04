using UnityEngine;

namespace ClothSimulation
{
    public class Fixer : MonoBehaviour
    {
        public bool IsInside(Vector3 pos)
        {
            return GetComponent<Collider>().bounds.Contains(pos);
        }
    }
}