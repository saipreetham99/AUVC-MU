using UnityEditor;

namespace StylizedWater3.DynamicEffects
{
    [CustomEditor(typeof(ShorelineWaveSpawner))]
    public class ShorelineWaveSpawnerInspector : Editor
    {
        public override void OnInspectorGUI()
        {
            UI.DrawHeader();
            
            #if !SPLINES
            UI.DrawNotification("This component requires the \"Splines\" package to be installed", MessageType.Error);
            #else
            base.OnInspectorGUI();
            #endif
            
            UI.DrawFooter();
        }
    }
}